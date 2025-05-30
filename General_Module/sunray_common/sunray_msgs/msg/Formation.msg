# Formation.msg
# 发布指令到formation节点

## 指令类型 takeoff, land, hover, formation
uint8 cmd
uint8 TAKEOFF=0
uint8 LAND=1
uint8 HOVER=2
uint8 FORMATION=3

uint8 formation_type
uint8 STATIC=0
uint8 DYNAMIC=1

string name