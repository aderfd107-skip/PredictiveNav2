# 02：创建 prediction 包骨架

新建 `predictive_nav_prediction` C++ 包，只放一个最小节点和正确依赖，暂时不做预测。

**包**是 ROS 2 中可单独构建、安装和运行的一组代码。将预测独立出来，未来能单独测试、回放或替换它，而不会把代码塞进 tracking 节点。

完成标准：`colcon build --packages-select predictive_nav_prediction` 成功，节点能输出启动日志。
