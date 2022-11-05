# 发送者名称
string name # id+type
# 数据来源信息
int32 id # 一般是自己id，也可以是别人的id
string type
# 队形中心信息
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# 邻接表信息
float64[] adjacency # 邻接表（起始于0直到已知id最大）