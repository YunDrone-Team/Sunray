# RTKOrigin.msg - RTK原点设置消息（用于地面站设置所有无人机的统一原点）
# 话题: /sunray/global_rtk_origin (所有无人机订阅同一个话题)

Header header
float64 latitude     # 原点纬度 [度]，范围: -90 ~ 90
float64 longitude    # 原点经度 [度]，范围: -180 ~ 180
float64 altitude     # 原点高度 [米]，海拔高度(AMSL)
