# pointcloud_to_laserscan

下载好后，直接编译即可通过，注释已经写好，可以自己在launch文件里修改参数

想要在omtb里面使用此包，进行点云转换，需要在amcl.launch里添加重映射
```
<remap from="scan" to="tfscan"/>
```
然后在导航launch文件里将此节点添加进去，便可一并使用
```
<!-- 点云转换 -->
<include file="$(find pointcloud_to_laserscan)/launch/pointcloud_to_laserscan.launch">
</include>
```
