# -*- coding: utf-8 -*-
from pathlib import Path
p = Path(r"c:\Users\XiaoXin\Desktop\BG_4G\beidou_4g\bsp\AIR780EP.cpp")
t = p.read_text(encoding="utf-8")
repls = [
    ('logWarning("LBS skip: AGNSS active");', 'logWarning("LBS: 跳过(AGNSS收包中)");'),
    ('logWarning("LBS timeout (%ums)", (unsigned) timeout_ms);',
     'logWarning("LBS: 超时 %ums", (unsigned) timeout_ms);'),
    ('logWarning("LBS fail code=%u", (unsigned) out->code);',
     'logWarning("LBS: 失败 code=%u", (unsigned) out->code);'),
    ('logInfo("LBS ok: lat=%f lon=%f", out->latitude, out->longitude);',
     'logInfo("LBS: 成功 lat=%f lon=%f", out->latitude, out->longitude);'),
]
for a, b in repls:
    if a in t:
        t = t.replace(a, b)
        print("ok", b[:30])
    else:
        print("miss", a[:40])
p.write_text(t, encoding="utf-8")

# delete this script too
Path(r"c:\Users\XiaoXin\Desktop\BG_4G\beidou_4g\tools\_finish_logs.py").unlink(missing_ok=True)
print("done")
