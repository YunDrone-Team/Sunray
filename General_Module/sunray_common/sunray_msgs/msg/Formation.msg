# Formation.msg
# 发布指令到formation节点

## 指令类型 takeoff, land, hover, formation
uint8 cmd
uint8 INIT=0
## 起飞
uint8 TAKEOFF=1
## 降落
uint8 LAND=2
## 悬停
uint8 HOVER=3
## 设置编队
uint8 FORMATION=4
## 设置home点
uint8 SET_HOME=5
## 返回home点
uint8 RETURN_HOME=6

uint8 formation_type
uint8 GOAL=0
uint8 STATIC=1
uint8 DYNAMIC=2
uint8 LEADER=3
uint8 FLLOW=4

string name