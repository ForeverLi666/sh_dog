# 原始机器人资产

本目录保存当前 `URDF-V4-test1` CAD 导出包中规范化所需的原始输入：

- `urdf/URDF-V4-test1.urdf`
- `meshes/*.STL`

这些文件保持导出时的名称和内容，不直接修改。旧版本由 Git 历史保留，规范化 URDF 直接引用此处
唯一一套 STL。
USD、MJCF 等后端模型由规范化 URDF 继续生成，不得反向成为事实来源。

原导出包中的 ROS launch、package 元数据、关节名示例、CSV 和导出日志不属于机器人几何与动力学输入，因此未纳入仓库。
