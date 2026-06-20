#!/usr/bin/env python3
"""
upload_bins.py — IRext AtomS3 独立枚举器配套工具

两个子命令：

  prepare   从 IRext 数据库筛选某品牌的 bin 文件，复制到 data/ 目录
            （文件名会被重命名为 <catId>_<subCat>_<原名>.bin，
             以便固件按文件名解析 catId/subCat）
            复制完后用 `pio run -t uploadfs` 把 data/ 烧进 LittleFS，
            之后 AtomS3 断电也能离线枚举（模式A）。

  remote    不烧录，电脑直接通过 TCP 把 bin 一条条推给 AtomS3（模式B）。
            适合先在桌面快速试，不想每次都重新刷 LittleFS。

用法：
  # 准备：把格力空调所有 bin 复制并改名到 ./data
  python3 upload_bins.py prepare \
      --db irext_db_20260519_sqlite3.db \
      --bin-dir irext-binaries_20260519/irext-binaries_20260519 \
      --brand 格力 --cat 1 \
      --out data

  # 然后：
  cd <项目目录>
  pio run -t uploadfs

  # 或者：不烧录，直接联机推送枚举（电脑需在 AtomS3 AP 范围内或同一局域网）
  python3 upload_bins.py remote \
      --db irext_db_20260519_sqlite3.db \
      --bin-dir irext-binaries_20260519/irext-binaries_20260519 \
      --brand TCL --cat 0 \
      --host 192.168.4.1 --type tv --keycode 0
"""

import sqlite3
import argparse
import os
import sys
import glob
import shutil
import socket
import base64
import json
import time

TCP_TIMEOUT = 8

# ── 数据库查询（与 irext_enum.py 相同逻辑） ───────────────────────────────────

def get_candidates(db_path, brand, category_id):
    if not os.path.exists(db_path):
        print(f"[错误] 数据库文件不存在: {os.path.abspath(db_path)}")
        print(f"       请检查 --db 参数路径是否正确（可以用绝对路径）")
        sys.exit(1)
    conn = sqlite3.connect(db_path)
    cur = conn.execute("""
        SELECT ri.id, ri.brand_name, ri.protocol, ri.remote,
               ri.category_id, ri.sub_cate, ri.status
        FROM remote_index ri
        JOIN brand b ON ri.brand_id = b.id
        WHERE (ri.brand_name LIKE ? OR b.name_en LIKE ?)
          AND ri.category_id = ?
          AND ri.status = 1
        ORDER BY ri.id
    """, (f"%{brand}%", f"%{brand}%", category_id))
    rows = cur.fetchall()
    conn.close()
    return rows

def find_bin(bin_dir, protocol, remote):
    patterns = [
        os.path.join(bin_dir, f"irda_{protocol}_{remote}.bin"),
        os.path.join(bin_dir, f"irda_{protocol}.bin"),
        os.path.join(bin_dir, f"{protocol}_{remote}.bin"),
        os.path.join(bin_dir, f"{protocol}.bin"),
    ]
    for pat in patterns:
        m = glob.glob(pat)
        if m: return m[0]
        m = glob.glob(os.path.join(bin_dir, "**", os.path.basename(pat)), recursive=True)
        if m: return m[0]
    return None

# ── 子命令: prepare ───────────────────────────────────────────────────────────

def cmd_prepare(args):
    print(f"[*] 查询品牌「{args.brand}」category_id={args.cat} ...")
    candidates = get_candidates(args.db, args.brand, args.cat)
    if not candidates:
        print("[-] 未找到匹配条目")
        sys.exit(1)

    os.makedirs(args.out, exist_ok=True)

    copied, missing = 0, 0
    manifest = []

    for i, (rid, brand, protocol, remote, cat, sub_cate, status) in enumerate(candidates, 1):
        src = find_bin(args.bin_dir, protocol, remote)
        if not src:
            missing += 1
            continue
        # 固件按文件名前缀 "<catId>_<subCat>_" 解析参数
        # LittleFS 文件名长度有限（约32字节含路径'/'），protocol 名过长会导致
        # mklittlefs 报 "unable to open" / "error adding file"，
        # 所以这里只保留 cat_sub_序号，完整 protocol/remote 信息写进 _manifest.txt
        dst_name = f"{cat}_{sub_cate}_{i:03d}.bin"
        dst = os.path.join(args.out, dst_name)
        if len(dst_name) > 28:
            print(f"  [警告] 文件名仍然偏长，可能在某些 LittleFS 版本上失败: {dst_name}")
        shutil.copy2(src, dst)
        copied += 1
        manifest.append((dst_name, rid, cat, sub_cate, protocol, remote))
        print(f"  [{copied}] {dst_name}  ← {protocol}_{remote}")

    print(f"\n[*] 完成：复制 {copied} 个，缺失 {missing} 个")
    print(f"[*] 文件已放入: {os.path.abspath(args.out)}")

    # 清单写在 data/ 外面，避免占用 LittleFS 空间（固件只认 .bin，但仍会占 flash）
    manifest_path = os.path.join(os.path.dirname(args.out.rstrip("/\\")) or ".", "_manifest.txt")
    with open(manifest_path, "w", encoding="utf-8") as f:
        f.write(f"品牌: {args.brand}  category_id: {args.cat}\n")
        f.write(f"{'文件名':<16} {'db_id':<8} {'cat':<4} {'sub':<4} {'protocol':<24} remote\n")
        for name, rid, cat, sub, proto, remote in manifest:
            f.write(f"{name:<35} {rid:<8} {cat:<4} {sub:<4} {proto:<20} {remote}\n")
    print(f"[*] 对照清单: {manifest_path}")

    print(f"\n下一步：")
    print(f"  1. 确认 {args.out}/ 目录在你的 PlatformIO 项目根目录下")
    print(f"  2. 运行: pio run -t uploadfs")
    print(f"  3. 烧录后 AtomS3 重启会自动进入独立枚举模式（模式A）")

# ── 子命令: remote（模式B，复用 TCP 协议） ────────────────────────────────────

def b64enc(data: bytes) -> str:
    return base64.b64encode(data).decode()

def tcp_send_recv(sock, msg: str) -> str:
    sock.sendall((msg.strip() + "\n").encode())
    sock.settimeout(TCP_TIMEOUT)
    resp = b""
    while True:
        try:
            chunk = sock.recv(512)
            if not chunk: break
            resp += chunk
            if b"\n" in resp: break
        except socket.timeout:
            break
    return resp.decode(errors="ignore").strip()

def make_payload(args):
    if args.type == "ac":
        return {
            "keyCode": 0,
            "acStatus": {
                "acPower": 0, "acTemp": args.temp - 16, "acMode": args.mode,
                "acWindDir": 0, "acWindSpeed": args.fan,
                "acDisplay": 0, "acSleep": 0, "acTimer": 0, "changeWindDir": 0,
            }
        }
    else:
        return {"keyCode": args.keycode}

def send_one(host, port, bin_path, cat, subcat, payload):
    try:
        sock = socket.create_connection((host, port), timeout=TCP_TIMEOUT)
    except Exception as e:
        return False, f"连接失败: {e}"
    try:
        resp = tcp_send_recv(sock, "a_hello")
        if "e_hello" not in resp:
            return False, f"握手失败: {resp}"

        with open(bin_path, "rb") as f:
            bin_data = f.read()
        b64_bin = b64enc(bin_data)
        resp = tcp_send_recv(sock, f"a_bin,{cat},{subcat},{len(b64_bin)},{b64_bin}")
        if "e_bin" not in resp:
            return False, f"bin加载失败: {resp}"

        ctrl_json = json.dumps(payload, separators=(",", ":"))
        b64_ctrl  = b64enc(ctrl_json.encode())
        resp = tcp_send_recv(sock, f"a_control,{len(b64_ctrl)},{b64_ctrl}")
        return ("e_success" in resp), resp
    except Exception as e:
        return False, f"异常: {e}"
    finally:
        sock.close()

def cmd_remote(args):
    print(f"[*] 查询品牌「{args.brand}」category_id={args.cat} ...")
    candidates = get_candidates(args.db, args.brand, args.cat)
    if not candidates:
        print("[-] 未找到匹配条目"); sys.exit(1)

    entries = []
    for rid, brand, protocol, remote, cat, sub_cate, status in candidates:
        bp = find_bin(args.bin_dir, protocol, remote)
        if bp:
            entries.append((rid, protocol, remote, cat, sub_cate, bp))

    print(f"[*] 共 {len(entries)} 个候选\n")
    payload = make_payload(args)
    print(f"[*] 推送目标: {args.host}:{args.port}  指令: {payload}\n")

    input("按回车开始（AtomS3 Web 终端会同步显示日志，建议同时打开）...")

    hits = []
    for i, (rid, protocol, remote, cat, sub_cate, bp) in enumerate(entries, 1):
        print(f"[{i}/{len(entries)}] db_id={rid} subcat={sub_cate} {protocol}_{remote}")
        ok, msg = send_one(args.host, args.port, bp, cat, sub_cate, payload)
        print(f"   {'✅' if ok else '⚠️ '} {msg}")

        ans = input("   有反应吗? [y=成功 / q=退出 / 回车=下一个]: ").strip().lower()
        if ans == "y":
            hits.append((rid, sub_cate, protocol, remote))
            print("   🎯 已记录")
        elif ans == "q":
            break

    if hits:
        print("\n🎯 命中列表：")
        for rid, sub_cate, protocol, remote in hits:
            print(f"   db_id={rid} subcat={sub_cate} irda_{protocol}_{remote}.bin")

# ── 入口 ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="IRext AtomS3 独立枚举器配套工具",
                                  formatter_class=argparse.RawDescriptionHelpFormatter,
                                  epilog=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)

    p1 = sub.add_parser("prepare", help="复制 bin 到 data/ 目录，供 uploadfs 烧录（模式A）")
    p1.add_argument("--db", required=True)
    p1.add_argument("--bin-dir", required=True)
    p1.add_argument("--brand", required=True)
    p1.add_argument("--cat", type=int, required=True)
    p1.add_argument("--out", default="data")
    p1.set_defaults(func=cmd_prepare)

    p2 = sub.add_parser("remote", help="不烧录，TCP 实时推送枚举（模式B）")
    p2.add_argument("--db", required=True)
    p2.add_argument("--bin-dir", required=True)
    p2.add_argument("--brand", required=True)
    p2.add_argument("--cat", type=int, required=True)
    p2.add_argument("--host", default="192.168.4.1", help="AtomS3 IP（AP默认网关）")
    p2.add_argument("--port", type=int, default=8080)
    p2.add_argument("--type", choices=["ac", "tv"], default="tv")
    p2.add_argument("--keycode", type=int, default=0)
    p2.add_argument("--temp", type=int, default=26)
    p2.add_argument("--mode", type=int, default=0)
    p2.add_argument("--fan", type=int, default=0)
    p2.set_defaults(func=cmd_remote)

    args = ap.parse_args()
    args.func(args)

if __name__ == "__main__":
    main()
