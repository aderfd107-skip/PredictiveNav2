# 06：录制 rosbag

记录 `/scan`、`/odom`、clusters、tracks、predictions、控制命令、导航结果和必要 TF，命名关联到场景和配置版本。

**rosbag** 是话题录像；它保存的是证据和可回放输入，不是只用于出错后临时抓日志。

完成标准：一次实验对应一个可追溯 bag，知道如何检查其 topic 列表和时长。
