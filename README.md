# vtc_unity
[![vtc_unity](.image/vtc.gif)](https://youtu.be/iVK--llmc00)  
![kashiwanoha](.image/kashiwanoha.gif)  
Virtual Tsukuba Challenge on Unity.  

[The Virtual Tsukuba Challenge (VTC) using Unreal Engine](https://github.com/furo-org/VTC) has been developed by [fuRo](https://www.furo.org/).

This package is the Unity version of VTC.  
In this repository, following packages are utilized.  
Please check them if you have more interests.

- [vtc_world_unity](https://github.com/Field-Robotics-Japan/vtc_world_unity) : The Tsukuba Challenge (environment) model.
- [kashiwanoha_world_unity](https://github.com/Field-Robotics-Japan/kashiwanoha_world_unity) : The Kashiwa No Ha Campus (environment) model.
- [unit04_unity](https://github.com/Field-Robotics-Japan/unit04_unity) : The robot (unit04) model.
- [sensors_unity](https://github.com/Field-Robotics-Japan/sensors_unity) : The sensor models.

# Installation
### 1．Installation of unity(2019.3.10f1)
First, install UnityHub with following links.
- Windows, Mac : https://unity3d.com/jp/get-unity/download
- Linux : Goto https://unity3d.com/get-unity/download and click "Download Unity Hub" button to get latest `UnityHub.AppImage`.  
  Then add execution permission for `UnityHub.AppImage` by following command.
  ```bash
  $ sudo chmod +x UnityHub.AppImage
  ```
  Then, run the UnityHub.AppImage
   ```bash
   $ ./UnityHub.AppImage
   ```
   Please certificate the LICENSE for Unity on UnityHub application (you can use them free !)

After that, choose and install Unity Editor (version : `2019.3.10f`) from archive.  
https://unity3d.com/get-unity/download/archive

**For Windows**  
You have `.exe` file from above link. Just run them.

**For Linux**  
1. Right click on `Unity Hub` button on your desired Unity Editor version, and click "Copy Lilnk Location".
2. Run `UnityHub.AppImage` by setting copied link location as the argument. Here is the example for `2019.3.10` version.
   ```bash
   $ ./UnityHub.AppImage unityhub://2019.3.10f1/5968d7f82152
   ```
   If you need any other version, the procedure is same.
   After above commands, the UnityHub will start to install desird version's Unity Editor!

### 4.Open vtc_unity
Finally, please open `vtc_unity` package from UnityHub. (It takes more than 5 minuites at the first time, in the case)

### 5. Select the Scene file
There are two Scene files in `Asset/vtc_unity/Scene/` directory.  
Please open the Scene file you want.

### Trouble Shooting
#### Trouble with git lfs
Some problems are come from `git lfs`
- If you have not installed `git lfs`, please install them. Then, `git clone` this repository again.
- If you already have installed `git lfs`, but have a problem. Try following procedure please.
```bash
$ git clean -fdx
$ git lfs pull
```

#### Trouble with Blender
The old version of this repository, you need to install blender for launch.  
Thanks to [@ssilph](https://github.com/ssilph) from [#8](https://github.com/Field-Robotics-Japan/vtc_world_unity/issues/8)
##### 1．Installation of Blender 2.8x
You need Blender>=2.8x. Plsease install with following commands (for Ubuntu).
```bash
$ sudo apt remove blender
$ sudo add-apt-repository ppa:thomas-schiex/blender
$ sudo apt update
$ sudo apt install blender
```
##### 2．Installation of PointCloudViewer(version=2.8x) + bpy(version=2.8x)
You also need, PointCloudViewer and bpy.
First, download zip file from following link.
- https://github.com/uhlik/bpy#point-cloud-visualizer-for-blender-280  
Then, install them with following procedure.
- blender->Edit(Top Left)->Preferences->Add-ons(Left side)->install(Top Right)->choose zip file->install Add-on
Extract zip file

Install `space_view3d_point_cloud_visualizer.py` with same procedure.
- blender->Edit(Top Left)->Preferences->Add-ons(Left side)->install(Top Right)->choose space_view3d_point_cloud_visualizer.py file->install Add-on


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
- [Stencil2 PointCloud data](https://github.com/Field-Robotics-Japan/vtc_world_unity/tree/master/Assets/PointCloud) : [Apache2.0](http://www.apache.org/licenses/LICENSE-2.0
) by 防衛大学校ソフトウェア工学講座  
  This data is referenced to create the world model.
- [City Hall.fbx](./Assets/Rawdata) : [Apache2.0](http://www.apache.org/licenses/LICENSE-2.0
) by [Tomoaki Yoshida](https://github.com/furo-org/VTC)  
  [City Hall.prefab](./Assets/Prefab) is modified from above FBX model.
- [FBX data for Kashiwa No Ha Campus](https://github.com/Field-Robotics-Japan/kashiwanoha_world_unity/Assets/kashiwanoha_world_unity/FBX) : [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) by [National Institute of Advanced Industrial Science and Technology （AIST）](https://www.aist.go.jp/)  
  
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
