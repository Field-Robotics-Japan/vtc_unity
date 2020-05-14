# vtc_unity
[![vtc_unity](.image/vtc.gif)](https://youtu.be/iVK--llmc00)  
Virtual Tsukuba Challenge on Unity.  

The name "Virtual Tsukuba Challenge (VTC)" was invented by [fuRo](https://www.furo.org/).  
The fuRo team maintains [Unrial Engine version of VTC](https://github.com/furo-org/VTC2018).   
If you have interests on that, please vitsit above link.  

This package is the Unity version of VTC.  
In this repository, following packages are utilized.  
Please check them if you have more interests.

- [vtc_world_unity](https://github.com/Field-Robotics-Japan/vtc_world_unity) : The world (environment) model.
- [unit04_unity](https://github.com/Field-Robotics-Japan/unit04_unity) : The robot (unit04) model.
- [sensors_unity](https://github.com/Field-Robotics-Japan/sensors_unity) : The sensor models.

# How to use
## 1. Launch ROS packages
#### 1-1 rosbridge
Launch the `rosbridge` with following command.
```bash
$ roslaunch rosbridge_server rosbridge_websocket.launch address:=localhost
```
#### 1-2 velodyne_pointcloud
Launch the `velodyne_pointcloud` package with following launch file.  
Please create launch file by copy and paste following script.
```xml
<launch>
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP16db.yaml" />
  <arg name="manager" default="velodyne_pointcloud" />
  <arg name="max_range" default="100.0" />
  <arg name="min_range" default="0.9" />

  <node pkg="velodyne_pointcloud" type="cloud_node" name="$(arg manager)">
    <param name="model" value="VLP16"/>
    <param name="calibration" value="$(arg calibration)"/>
    <param name="max_range" value="$(arg max_range)"/>
    <param name="min_range" value="$(arg min_range)"/>
  </node>
</launch>
```
        
## 2. Clone and run on Unity
#### 2-1
Just to clone this repo with `git clone` command.
#### 2-2
Then, open the project file with UnityHub.
#### 2-3
Finally, RUN the scene file named `unit04_test`.

# LICENSE Dependencies
- [PointCloud data](https://github.com/Field-Robotics-Japan/vtc_world_unity/tree/master/Assets/PointCloud) : [Apache2.0](http://www.apache.org/licenses/LICENSE-2.0
) by [Tetsuo Tomizawa](https://researchmap.jp/read0115628)
- [City Hall.fbx](./Assets/Rawdata) : [Apache2.0](http://www.apache.org/licenses/LICENSE-2.0
) by [Tomoaki Yoshida](https://github.com/furo-org/VTC2018)  
  [City Hall.prefab](./Assets/Prefab) is modified from above FBX model.
### From Unity Asset store
We utilize following Assets from Unity Asset Store.  
Every package are Free now (2020/05/13).
We partly modify and utilize those Assets.
- [RosSharp](https://github.com/siemens/ros-sharp) : [Apache2.0](http://www.apache.org/licenses/LICENSE-2.0)
- [Realistic Tree 9](https://assetstore.unity.com/packages/3d/vegetation/trees/realistic-tree-9-rainbow-tree-54622)
- [Mobile Tree Package](https://assetstore.unity.com/packages/3d/vegetation/trees/mobile-tree-package-18866)
- [Yughues Free Bushes](https://assetstore.unity.com/packages/3d/vegetation/plants/yughues-free-bushes-13168)
- [Yughues Free Ground Materials](https://assetstore.unity.com/packages/2d/textures-materials/floors/yughues-free-ground-materials-13001)
- [PBR Tile Texture Floor](https://assetstore.unity.com/packages/2d/textures-materials/pbr-tile-texture-floor-36243)
- [Birch Tree Pack vol. 1](https://assetstore.unity.com/packages/3d/vegetation/trees/birch-tree-pack-vol-1-49093)
- [Forester Pro Sycamore](https://assetstore.unity.com/packages/3d/vegetation/trees/forester-pro-sycamore-5980)
- [Asphalt materials](https://assetstore.unity.com/packages/2d/textures-materials/roads/asphalt-materials-141036)
- [Easy Grass Substance](https://assetstore.unity.com/packages/2d/textures-materials/floors/easy-grass-substance-82272)
- [First Person All-in-One](https://assetstore.unity.com/packages/tools/input-management/first-person-all-in-one-135316)
- [Simple Cars Pack](https://assetstore.unity.com/packages/3d/vehicles/land/simple-cars-pack-97669)
- [UAA - City Props - Vehicles](https://assetstore.unity.com/packages/3d/vehicles/land/uaa-city-props-vehicles-120339)
- [Stylized Vehicles Pack - FREE](https://assetstore.unity.com/packages/3d/vehicles/land/stylized-vehicles-pack-free-150318)

# LICENSE
Copyright [2020] Ryodo Tanaka groadpg@gmail.com

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
