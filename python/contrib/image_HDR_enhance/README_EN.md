开发者板（Atlas200DK）连接一指禅
<!-- GFM-TOC -->
* [宝藏一:开发板连接俩步走](#宝藏一开发板连接俩步走)
    * [确定开发板的连接方式](#确定你要通过什么方式连接开发板)
    * [按选定的连接方式进行硬件连接配置](#请按照第一步选择的连接方式进行硬件连接配置)
* [宝藏二:开发板链接问题定位指南](#宝藏二开发板链接问题定位指南)
    * [环境检查](#1-环境检查)
    * [无USB虚拟网卡](#1-1无USB虚拟网卡)
    * [有USB虚拟网卡无IP](#1-2IP没有起来)
    * [USB虚拟网卡和IP均已起来，但ping不通192.168.1.2](#1-3ping不通IP)

<!-- GFM-TOC -->

本指导主要是用于帮助您快速将开发者板（Atlas200DK，下文统称开发板）和主机连接，这里的主机指的是你用来做开发环境的机器，可以是个人便携/台式机也可以是服务器；本文将包括两个宝藏，宝藏一告诉你如何连接开发板，宝藏二告诉你如何定位开发板连接失败问题，以及常见求助渠道。


# 宝藏一开发板连接俩步走

### 确定你要通过什么方式连接开发板

<a name="table32057793"></a>
<table><tbody><tr id="row18281552"><td class="cellrowborder" valign="top" width="11%"><p id="p4410728"><a name="p4410728"></a><a name="p4410728"></a>连接方式</p>
</td>
<td class="cellrowborder" valign="top" width="25.510204081632654%"><p id="p21724664"><a name="p21724664"></a><a name="p21724664"></a>连接原理</p>
</td>
<td class="cellrowborder" valign="top" width="19.387755102040817%"><p id="p14867369"><a name="p14867369"></a><a name="p14867369"></a>优点</p>
</td>
<td class="cellrowborder" valign="top" width="36.734693877551024%"><p id="p63406242"><a name="p63406242"></a><a name="p63406242"></a>缺点</p>
</td>
<td class="cellrowborder" valign="top" width="8.16326530612245%"><p id="p35632012"><a name="p35632012"></a><a name="p35632012"></a>建议</p>
</td>
</tr>
<tr id="row52252659"><td class="cellrowborder" valign="top" width="11%"><p id="p4606985"><a name="p4606985"></a><a name="p4606985"></a>USB</p>
</td>
<td class="cellrowborder" valign="top" width="25%"><p id="p37621475"><a name="p37621475"></a><a name="p37621475"></a>USB type-c线插入主机时，主机上会出现一个虚拟网卡，将该虚拟网卡IP与开发板上的usb0网卡IP配置为同一网段即可通讯</p>
</td>
<td class="cellrowborder" valign="top" width="20%"><p id="p27440617"><a name="p27440617"></a><a name="p27440617"></a>USB type-c线默认配备，主机上一般都有USB口，硬件条件极易满足。</p>
</td>
<td class="cellrowborder" valign="top" width="37%"><p id="p8097541"><a name="p8097541"></a><a name="p8097541"></a>USB虚拟网卡方案不易理解，USB虚拟网卡不太稳定，IP容易掉，插入主机上不同的usb接口时，虚拟网卡名称会变化，需要修改配置；有时候需要经常重启网络服务；在不同的操作系统下容易有异常情况出现；开发板配置连网（公网）稍复杂；</p>
</td>
<td class="cellrowborder" valign="top" width="9%">&nbsp;&nbsp;</td>
</tr>
<tr id="row64636290"><td class="cellrowborder" valign="top" width="11%"><p id="p1048160"><a name="p1048160"></a><a name="p1048160"></a>网线</p>
</td>
<td class="cellrowborder" valign="top" width="25%"><p id="p17792110"><a name="p17792110"></a><a name="p17792110"></a>将主机侧用于与连接开发板连接的网卡（网口）IP和开发板上的eth0网卡IP设置为同一网段即可通讯</p>
</td>
<td class="cellrowborder" valign="top" width="20%"><p id="p31874772"><a name="p31874772"></a><a name="p31874772"></a>网线连接，直观稳定，容易理解</p>
</td>
<td class="cellrowborder" valign="top" width="37%"><p id="p31719739"><a name="p31719739"></a><a name="p31719739"></a>需要主机有多余的网卡（网口），一般个人便携较难满足；</p>
<p id="p17042198"><a name="p17042198"></a><a name="p17042198"></a>开发板无法连公网；</p>
</td>
<td class="cellrowborder" valign="top" width="9%">&nbsp;&nbsp;</td>
</tr>
<tr id="row8622890"><td class="cellrowborder" valign="top" width="11%"><p id="p27365507"><a name="p27365507"></a><a name="p27365507"></a>网线+路由器+dhcp</p>
</td>
<td class="cellrowborder" valign="top" width="25%"><p id="p2013564"><a name="p2013564"></a><a name="p2013564"></a>将开发板通过网线与路由器连接（你的主机也通过该路由器联网），只要配置开发板上的eth0网卡为自动获取IP（确保你的路由器DHCP打开），那么开发板与你的主机在同一局域网</p>
</td>
<td class="cellrowborder" valign="top" width="20%"><p id="p28880957"><a name="p28880957"></a><a name="p28880957"></a>配置简单，直观稳定，开发板与主机可分离；开发板可以直接联网（公网），主机可移动</p>
</td>
<td class="cellrowborder" valign="top" width="37%"><p id="p57656175"><a name="p57656175"></a><a name="p57656175"></a>需要你有路由器，有网线（不过现在基本都有路由器，强烈推荐该方式）；</p>
<p id="p49143529"><a name="p49143529"></a><a name="p49143529"></a>需要先usb方式先连接上开发板才能对开发板做配置，但只需要做一次即可。</p>
</td>
<td class="cellrowborder" valign="top" width="9%"><p id="p21202951"><a name="p21202951"></a><a name="p21202951"></a>优先建议</p>
</td>
</tr>
<tr id="row56608836"><td class="cellrowborder" valign="top" width="11%"><p id="p21913016"><a name="p21913016"></a><a name="p21913016"></a>Wifi（为开发板配置无线模块）</p>
</td>
<td class="cellrowborder" valign="top" width="26%"><p id="p30123835"><a name="p30123835"></a><a name="p30123835"></a>为开发板配置一个wifi外挂（例如huawei ws331a)，开发板与ws331a之间用网线连接，需要与开发板通讯的设备（PC机或者手机）无线连接ws331a的wifi</p>
</td>
<td class="cellrowborder" valign="top" width="20%"><p id="p24111608"><a name="p24111608"></a><a name="p24111608"></a>开发板和主机均可移动；开发板可以直接连公网；</p>
</td>
<td class="cellrowborder" valign="top" width="37%"><p id="p6883221"><a name="p6883221"></a><a name="p6883221"></a>需要你购买额外的迷你路由器；</p>
<p id="p61948991"><a name="p61948991"></a><a name="p61948991"></a>且仅当路由器有中继功能时开发板才能上网（公网）；配置稍复杂</p>
</td>
<td class="cellrowborder" valign="top" width="9%">&nbsp;&nbsp;</td>
</tr>
</tbody>
</table>

# 请按照第一步选择的连接方式进行硬件连接配置

### 1.1  USB
将usb 线type-c一端插入开发板，另外一端插入主机；
然后可以主机上通过ifconfig（如果是windows，请查看控制面板中的查看虚拟出来的虚拟网卡名称）查看虚拟网卡名称（一般网卡名字为比较复杂的一个字符串，例如enp0s20f0u4，注意在主机上插入不同的usb接口，该名称会有变化）；
如果虚拟网卡根本就出不来，请根据宝藏二定位问题并解决；
配置该虚拟网卡IP为192.168.1.xxx（因为开发板上usb0网卡默认IP是192.168.1.2），并重启网络服务；

\(**注意**：如果不会配置请[USB虚拟网卡IP配置](https://gitee.com/ascend/samples/wikis/USB%E8%99%9A%E6%8B%9F%E7%BD%91%E5%8D%A1IP%E9%85%8D%E7%BD%AE%EF%BC%88linux18.04%EF%BC%89?sort_id=3512359)

测试连接：
```
ssh  HwHiAiUser@192.168.1.2
```
如果能够登录，结束；
否则，根据宝藏二定位问题并解决。

### 1.2  网线
网线一端插入开发板网口，另外一端插入主机网口；
配置主机上被插入网口的IP为静态IP，静态IP请设置为192.168.0.xxx（开发板eth0默认IP为192.168.0.2，只需要保证主机侧IP与开发板eth0网卡IP为同一网段即可）。
测试连接：
```
ssh  HwHiAiUser@192.168.0.2
```
如果能够登录，结束；
否则，根据宝藏二定位问题并解决。

### 1.3 网线+路由器+dhcp
将开发板通过网线与路由器连接（你的主机也通过该路由器联网），只要配置开发板上的eth0网卡为自动获取IP（确保你的路由器DHCP打开），那么开发板与你的主机在同一局域网，便可通讯。
但需要先通过第一种方式暂时链接上开发板后才能配置开发板上的eth0网卡。
请参考：

[开发板联网的方法](https://gitee.com/ascend/samples/wikis/%E5%BC%80%E5%8F%91%E6%9D%BF%E8%81%94%E7%BD%91%E7%9A%84%E6%96%B9%E6%B3%95?sort_id=3511250)

如果能够登录开发板，结束；
否则，根据宝藏二定位问题并解决。

### 1.4 Wifi（为开发板配置无线模块）
为开发板配置一个wifi外挂（例如huawei ws331a\)，开发板与ws331a之间用网线连接，你需要与开发板通讯的设备（PC机或者手机）连上ws331a的wifi，即可与开发板通讯连接。
请参考:[Atlas200DK 配置wifi外挂模块](https://bbs.huaweicloud.com/blogs/134940)
如果能够登录开发板，结束；
否则，根据宝藏二定位问题并解决。

# 宝藏二开发板链接问题定位指南

首先确认你的使用场景，然后依据不同的连接方式进行定位，按照步骤走即可。
USB连接开发板场景

# 1 环境检查

请确保开发板已上电；
请确保开发板启动ok，4个LED全亮（早期的开发板是3个灯亮）；
然后查看usb虚拟网卡是否已经存在；
Linux系统请用ifconfig查看，windows系统请在控制面板中查看，均可以在开发板启动情况下通过插拔usb连线进行查看；
若虚拟网卡起不来，一直是无USB虚拟网卡状态，请转1.1；
若虚拟网卡起来了，但并没有分配IP，请转1.2

# 1-1无USB虚拟网卡

如果是用的virtualBox或者VMWARE虚拟机，请确保开发板是否与虚拟机链接\(例如vmware中是在菜单上进行查看：虚拟机-\>可移动设备\)，然后重试连接；
如果windows设备管理器中驱动有异常，则请更新RNDIS驱动；然后重试连接；
[RNDIS驱动更新](https://gitee.com/ascend/samples/wikis/%E5%BC%80%E5%8F%91%E6%9D%BF%E5%90%8E%E6%97%A0%E6%B3%95%E8%BF%9E%E6%8E%A5%E8%99%9A%E6%8B%9F%E6%9C%BA?sort_id=3512020)
请确保USB 数据线是好的，请尝试换根线试试，或者请尝试使用网线连接。若网线连接成功，则结束，否则请继续；

# 1-2IP没有起来

查看/etc/netplan/01-netcfg.yaml文件，确认虚拟网卡IP是否已配置、配置是否正确，注意对比网卡名称、IP和ifconfig -a查看到的虚拟网卡名称、IP是否一致，以及IP是否配置正确，与192.168.1.2是否在一个网段；
若01-netcfg.yaml文件中没有配置，请用root用户修改网络配置
```
vi /etc/netplan/01-netcfg.yaml
```
增加配置
```
 ethernets: 
    ...
    enp0s20f0u4:
      dhcp4: no
      addresses: [192.168.1.223/24]
      gateway4: 192.168.1.1
      nameservers:
            addresses: [255.255.0.0]
```
重启网络
```
netplan apply
```
再使用ifconfig -a查看IP是否已起来，然后ping 192.168.1.2，若ping不通，请继续。

# 1-3ping不通IP

###1.3.1 虚拟机场景，
请确保USB3.0兼容性已经打开，否则请关闭虚拟机后配置，再重新开启虚拟机，然后如果仍然无法连接，请继续；
![](figures/zh-cn_image_0231893186.png)
请排除IP冲突的可能行，请参考：[https://bbs.huaweicloud.com/forum/thread-16966-1-1.html](https://bbs.huaweicloud.com/forum/thread-16966-1-1.html)；
如果问题解决，则结束；

否则可能是开发板侧默认的usb0的静态IP被修改过，已经不是192.168.1.2，那请参考宝藏一2.2暂时换网线连接方式连接上后查看usb0默认IP，如果默认IP被修改了，那么请重新修改主机侧usb虚拟网卡IP地址为与usb0的IP为同一网段即可（当然也可以修改开发板usb0或者接受网线连接方式）。

### 1.3.2 网线连接场景
（主机有两个网口：主机通过其中一个网口上网，通过另外一个网口与开发板链接）
首先，请确认开发板和主机两端的网口和网线硬件均正常；
然后，请确认你配置了IP的主机侧网卡确实是你用于与开发板连接的网卡（可以通过插拔网线确认）；如果错了请修正，修正后连接正常则结束，否则请继续；
请确认你配置的主机侧IP与开发板eth0的IP在同一个网段（开发板eth0默认IP为192.168.0.2）；如果错了请修正，修正后连接正常则结束，否则请继续；
请排除IP冲突可能性，如果你配置的IP网段与你的局域网IP在同一个网段，请修改以避免IP冲突；修改方法：

第一步：将你用于上网的网线先拔掉，暂时先排除冲突，这时主机与开发板应该能连上了，请ssh登录到开发板；如果连不上考虑是开发板eth0的IP可能被改过了，请换用usb链接方式连接上查看并确认；
第二步：在开发板侧修改eth0的静态IP为其他网段，例如：192.168.11.2
第三步：在主机侧修改与开发板连接的网卡IP为与开发板eth0在同一网段，例如：192.168.11.129
第四步：重启开发板网络服务或者重启开发板；重启后，开发板应该能连接上了；
第五步：插上第一步拔掉的网线，恢复主机上网。

###1.3.3 网线连接场景（网线+路由器+dhcp）

首先，请确保主机与路由器是通的，能访问路由器页面；
然后，请登录路由器页面，查看有线连接的开发板IP，是否与你之前ssh登录的IP一致；一般情况下都是IP错了，请修改用正确的IP即可。
如果仍然无法解决问题，请换用其他方式连接；

###1.3.4 Wifi（为开发板配置无线模块）

可能是你购买的mini路由器没有中继功能；


#求助渠道

可以在wiki 常见问题定位中查找网络连接问题解决办法；
[wiki链接](https://gitee.com/ascend/samples/wikis)

可以在社区中查找解决方案或者提问题求助：
[社区论坛链接](https://bbs.huaweicloud.com/forum/forum-949-1.html)

可以在sample仓issue提问题求助：
[issue链接](https://gitee.com/ascend/samples/issues)
