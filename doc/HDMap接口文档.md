# HDMap Service

hdmap 工程中使用 Agent + Service 组合的方式进行工作，Service维护所有地图，提供必要的地图接口服务；Agent维护规划好的路线的地图，作为业务和接口连接者的存在。此外还可以使用MapView进行可视化，使用Test程序控制车辆移动，进行接口测试。



## Uages

直接调用将hdmap放置到catkin工作区的src目录下，与相应的业务进行配合使用

```
catkin_ws
|-- build
|-- devel
|-- src
    |-- HDMap
    |-- <your project>
```

在业务的CMakeLists.txt中添加`HDMap`，以使用提供的服务接口

```
find_package(catkin REQUIRED COMPONENTS
  ...
  HDMap
)
```



也可以直接使用自带的 `map_cmd.launch`进行请求，也可以自行发送`srv`进行请求。在使用业务之前，务必

```sh
# 在任何窗口使用HDMap前，需先加载环境
$ source devel/setup.sh

# 运行 Agent 和 Server
$ roslaunch HDMap hdmap.launch

# 运行地图控制台
$ roslaunch HDMap map_cmd.launch

# 开启map地图和实时位置展示
$ roslaunch HDMap map_view.launch
```





### 规划一条路到路的路线

在当前版本的HDMap中以路为单位进行规划



*自带控制台*

在 Map command 控制台输入， 满足n > 1 即可

```
r2r road1 road2 ... roadn
```



![1548320373256](HDMap接口文档/road_by_id.png)



发送server*

```c++
#include <HDMap/srv_map_cmd.h>
#include <HDMap/msg_route_info.h>
#include <vector>

void OnRouteInfoUpdate(const HDMap::msg_route_info& msg)
{
    Print(msg.route_id);
}

int main()
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<HDMap::srv_map_cmd>("map_command");
    ros::Subscriber　subscriber = n.subscribe("/map_pub_route_info",1,
                                             　OnRouteInfoUpdate);
    std::vector<int> v;
    
    HDMap::srv_map_cmd srv;
    srv.request.cmd = "r2r";
    srv.request.argv.push_back( road1 );
    srv.request.argv.push_back( road2 );
    ...
    if(client.call(srv))
    {
        v = srv.response.route;
    }
}
```



### 规划一条点到点的路线

> !!! 目前只支持两个点之间的规划 !!!



*自带控制台*

在 Map command 控制台输入起始点和终止点的坐标，起始点需要包含方向，终止点现在暂时没有考虑方向，所以只要求x2与y2 准确即可

```
p2p x1 y1 heading1 x2 y2 -
```



![1548320866088](HDMap接口文档/road_by_pos.png)



*发送server*

```c++
#include <HDMap/srv_map_cmd.h>
#include <vector>

void OnRouteInfoUpdate(const HDMap::msg_route_info& msg)
{
    Print(msg.route_id);
}

int main()
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<HDMap::srv_map_cmd>("map_command");
    ros::Subscriber　subscriber = n.subscribe("/map_pub_route_info",1,
                                             　OnRouteInfoUpdate);
    std::vector<int> v;
    
    HDMap::srv_map_cmd srv;
    srv.request.cmd = "p2p";
    srv.request.argv.push_back( -14.15 );
    srv.request.argv.push_back( -15.43 );
    srv.request.argv.push_back(  -35  );
    srv.request.argv.push_back(  642.58 ); 
    srv.request.argv.push_back(  -82.76 );
    srv.request.argv.push_back(  100 );
    if(client.call(srv))
    {
        v = srv.response.route;
    }
}
```



无论是何种方式进行服务的请求，服务只返回一个请求成功与否的布尔值，如果需要知道道路的信息，需要额外进行道路信息的订阅，主题为`map_pub_route_info` 的消息包含经过的路的坐标和路的编号。



### 地图仿真测试

加载环境后，运行

```sh
$ rosrun HDMap MapTest
```



![1548319020015](HDMap接口文档/MapTest01.png)



此外还请开启地图服务和map地图和实时位置展示

```shell
$: roslaunch HDMap hdmap.launch
$: roslaunch HDMap map_view.launch
```

此时RVIZ会打开，并自动加载资源，这时候使用MapTest进行车辆仿真控制。



![1548319435135](HDMap接口文档/MapTest02.png)



*绝对控制位置*

```sh
$: gps 0 0
```

方向值为缺省值，默认为0，当然可以直接指定，使用的单位是**角度**

```
$: gps 0 0 25
```



*WSAD控制*

可以使用 WSAD键控制仿真的移动，需要先输入命令

```sh
$: control
```

接着进入控制模式，此时使用WSAD分别进行上、下、原地左旋、原地右旋的操作，使用空格键加速，使用r键减速，注意不要超出道路，否则会有不好的后果。在仿真的过程中会有边缘点数量的打印，暂时使用这一打印信息判断服务器是否正常。

