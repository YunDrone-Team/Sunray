使用该代码（添加了自动起飞，搜索模式），需要做以下更改
（1）uav_search_strategy.launch
添加
```
# 在3行后面添加
<!-- 起飞指令，该指令为true，则只需要解锁就行，后续指令会自动执行；否则，则需要手动输入 -->
<arg name="takeoff" default="0"/> 

#在63行后面添加
<param name="takeoff" value="$(arg takeoff)"/>
```

（2）sunray_start.launch
```
# 在5行后面添加
<!-- 起飞指令，该指令为true，则只需要解锁就行，后续指令会自动执行；否则，则需要手动输入 -->
<arg name="takeoff" default="0"/> 

#在130行后面添加
<arg name="takeoff" value="$(arg takeoff)"/>

```
