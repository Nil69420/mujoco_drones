import re
import sys


def oracle_rows(header):
    text = open(header).read()
    block = re.search(r"#define MAVLINK_MESSAGE_CRCS \{(.*?)\}\}", text, re.S).group(1) + "}"
    rows = {}
    for match in re.finditer(r"\{(\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+)\}", block):
        msgid, extra, min_len, max_len, flags, ts, tc = (int(v) for v in match.groups())
        rows[msgid] = (extra, min_len, max_len, ts if flags & 1 else 255, tc if flags & 2 else 255)
    return rows


def generated_rows(source):
    rows = {}
    for match in re.finditer(r"^\s*\{(\d+), (\d+), (\d+), (\d+), (\d+), (\d+)\}", open(source).read(), re.M):
        msgid, extra, base, maxl, ts, tc = (int(v) for v in match.groups())
        rows[msgid] = (extra, base, maxl, ts, tc)
    return rows


def main():
    oracle = oracle_rows(sys.argv[1])
    ours = generated_rows(sys.argv[2])
    bad = 0
    for msgid in sorted(set(oracle) | set(ours)):
        if oracle.get(msgid) != ours.get(msgid):
            bad += 1
            print("MISMATCH", msgid, "oracle", oracle.get(msgid), "generated", ours.get(msgid))
    print("messages: oracle %d generated %d mismatches %d" % (len(oracle), len(ours), bad))
    sys.exit(1 if bad else 0)


main()
