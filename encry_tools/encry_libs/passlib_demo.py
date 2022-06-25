########################################################################################################################
# passlib 散列与验证 多次执行结果不同，使用 verify 来比较
# ------> 单向

from passlib.handlers.pbkdf2 import pbkdf2_sha256

passwd = pbkdf2_sha256.hash('123')
print(passwd)
print(pbkdf2_sha256.verify('123', passwd))
print(pbkdf2_sha256.verify('123', '$pbkdf2-sha256$29000$Osc4x3iPsZZy7n1PifF.rw$mK473W9l6zsLnS38rUaUpbEvaCoqXPJunfsroLeuVN0'))
print(pbkdf2_sha256.verify('123', '$pbkdf2-sha256$29000$.X9PqdV6DyHkXIux9l5rzQ$ZCv5bqMCW/HLBogcOB/grKhKsfDoTXwnNhEmwb9l8HI'))
