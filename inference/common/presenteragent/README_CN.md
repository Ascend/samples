中文|[英文](README.md)  
Presenter Agent安装步骤请参考[链接](../../environment/presenteragent_install)

Presenter部署在Mind Studio所在的Linux服务器上，主要作用是推理结果的展示。

Presenter包括Presenter Server与Presenter Agent。

-   Presenter Agent提供一系列API，用户通过调用API向Presenter Server推送媒体消息。
-   Presenter Server接收Presenter Agent发过来的数据，通过浏览器进行结果展示。

## Presenter Server<a name="zh-cn_topic_0147635264_section513113597483"></a>

-   **Description**

    Presenter Server是展示推理结果的软件包，该软件基于python3实现，并用到了第三方web框架tornado，以及底层通信框架protobuf。

    Presenter Server支持图片模式和视频模式。图片模式展示单张图片，视频模式以图片流的方式展示连续图片。Presenter server通过channel来标记不同的数据源，在浏览器里通过Create按钮添加channel，Delete进行删除，默认支持两路channel，分别是image和video。

-   **Sample Code**

    Presenter Server源码在样例的工程目录下，config是配置文件目录，修改config.conf进行可服务端ip和port的定制，logging.conf是logging模块的配置；src是源码目录，其中，presenter\_message\_pb2.py 定义protobuf格式，presenter\_socket\_server.py 负责并行接收数据，webapp.py 负责把数据推送到Chrome，进行前台展示；ui是web界面素材所在目录。

    与Presenter Agent的消息通信：

    Presenter Server 与Presenter Agent和Chrome的消息通信如下图所示，Chrome上发起创建channel的操作，Presenter Server发送数据到指定channel，Chrome打开此channel，观察推理结果。

    ![输入图片说明](https://images.gitee.com/uploads/images/2021/0112/114222_18033d3b_7401379.png "屏幕截图.png")

    Presenter Server与Presenter Agent之间消息结构如下，依次是4个字节的消息总长度，1个字节的消息名长度，若干字节的消息消息名，若干字节的protobuf内容。

    ```
    --------------------------------------------------------------
    |total message len      |    int             |    4 bytes                  |
    |--------------------------------------------------------------
    |message name len    |    byte           |    1 byte                   |
    |--------------------------------------------------------------
    |message name          |    string         |    xx bytes                |
    |--------------------------------------------------------------
    |message body           |    protobuf    |    xx bytes                |
    ---------------------------------------------------------------
    ```

    主要的消息有两个，一个是打开channel的请求消息OpenChannelRequest ，另一个是发送数据的消息PresentImageRequest ，其在ptorobuf中的定义如下：

    ```
    message OpenChannelRequest {
    string channel_name = 1; //channel 名称
    ChannelContentType content_type = 2; //数据模式，用来识别image和video
    }
    
    message PresentImageRequest {
    ImageFormat format = 1; // 图片格式，当前仅支持Jpeg
    uint32 width = 2; //图片宽度
    uint32 height =3; //图片高度
    bytes data = 4; //图片数据
    }
    ```

    通过epoll实现多路channel并行工作，实现伪码如下：

    ```
    def _server_listen_thread(self):
    """socket server thread, epoll listening all the socket events"""
    epoll = select.epoll()
    epoll.register(self._sock_server.fileno(), select.EPOLLIN | select.EPOLLHUP)
    try:
    conns = {}
    msgs = {}
    while True:
    events = epoll.poll(EPOLL_TIMEOUT)
    # timeout, but no event come, continue waiting
    if not events:
    continue
    for sock_fileno, event in events:
    # new connection request from presenter agent
    if self._sock_server.fileno() == sock_fileno:
    self._accept_new_socket(epoll, conns)
    # remote connection closed
    # it means presenter agent exit withot close socket.
    elif event & select.EPOLLHUP:
    self._clean_connect(sock_fileno, epoll, conns, msgs)
    # new data coming in a socket connection
    elif event & select.EPOLLIN:
    self._process_epollin(sock_fileno, epoll, conns, msgs)
    # receive event not recognize
    else:
    self._clean_connect(sock_fileno, epoll, conns, msgs)
    finally:
    epoll.unregister(self._sock_server.fileno())
    epoll.close()
    self._sock_server.close()
    ```

    消息解析过程，实现伪码如下，首先是解析消息，包括消息长度，消息名，最后读取protobuf并进行处理。

    ```
    def _read_sock_and_process_msg(self, sock_fileno, conns, msgs):
    # Step1: read msg head
    msg_total_len, msg_name_len = self._read_msg_head(sock_fileno, conns)
    if msg_total_len is None:
    return PRESENTER_ERR
    # Step2: read msg name
    msg_name = self._read_msg_name(conns[sock_fileno], msg_name_len)
    if msg_name == SOCKET_RECEIVE_NULL:
    return PRESENTER_ERR
    try:
    msg_name = msg_name.decode("utf-8")
    except UnicodeDecodeError:
    return PRESENTER_ERR
    # Step3:  read msg body
    msg_body_len = msg_total_len - MSG_HEAD_LENGTH - msg_name_len
    ret = self._read_msg_body(sock_fileno, conns, msgs, msg_name, msg_body_len)
    if ret == PRESENTER_ERR:
    return ret
    # Step4: process msg
    ret = self._process_msg(conns[sock_fileno], msg_name, msgs[sock_fileno])
    return ret
    ```

    解析protobuf，来自Presenter Agent的消息请求共有三个，分别是打开channel、发送数据、发送心跳。

    ```
    def _process_msg(self, conn, msg_name, msg_data):
    # process open channel request
    if msg_name == OPEN_CHANNEL_REQUEST_FULL_NAME:
    ret = self._process_open_channel(conn, msg_data)
    # process image request, receive an image data from presenter agent
    elif msg_name == PRESENT_IMAGE_REQUEST_FULL_NAME:
    ret = self._process_image_request(conn, msg_data)
    # process heartbeat request, it used to keepalive a channel path
    elif msg_name == HEART_BEAT_MESSAGE_FULL_NAME:
    ret = self._process_heartbeat(conn)
    else:
    ret = PRESENTER_ERR
    return ret
    ```


## Present Agent<a name="zh-cn_topic_0147635264_section41521616104018"></a>

Presenter Agent提供一系列API，用户可以调用这些API向Presenter Server推送媒体消息，并在浏览器中查看。当前支持JPEG格式图片的推送。

调用流程如下所示：

![输入图片说明](https://images.gitee.com/uploads/images/2021/0112/114246_8c1504f0_7401379.png "屏幕截图.png")

1.  App调用OpenChannel函数打开与Presenter Server间的通道。
2.  App调用SendMessage函数在该通道上推送媒体消息。推送消息时， 支持在推送的图片上画矩形框。使用时需要将框的左上、右下点的坐标、框的标题设置到PresentImageRequest对象中。
3.  所有图片发送完成后，App调用CloseChannel函数释放分配的资源。

-   **Sample Code**

    以发送图片为例：

    1.  <a name="zh-cn_topic_0147635264_li182791110135216"></a>Open channel

        ```
        OpenChannelParam param;
        param.hostIp = "127.0.0.1";  //IP address of Presenter Server
        param.port = 7006;  //port of present service
        param.channelName = "image";
        param.contentType = ContentType::kImage;  //content type is IMAGE
        
        Channel *channel = nullptr;
        PresenterErrorCode errorCode = OpenChannel(channel, param);
        if (errorCode != PresenterErrorCode::kNone) {
          return;
        }
        ```

    2.  SendMessage

        ```
        ascend::presenter::proto::PresentImageRequest request;
        request.set_data(string(reinterpret_cast<const char*>(buffer), size)); //image data buffer, image shuold be jpeg format
        request.set_width(1920);
        request.set_height(1280);
        
        //Set the rectangles info into request.
        ascend::presenter::proto::Rectangle_Attr *rectangle_attr = nullptr; 
        rectangle_attr = request.add_rectangle_list();   //Add one rectangle
        rectangle_attr->mutable_left_top()-> set_x(100); 
        rectangle_attr->mutable_left_top()-> set_y(100);
        rectangle_attr->mutable_right_bottom()->set_x(500);
        rectangle_attr->mutable_right_bottom()->set_y(500);
        rectangle_attr->set_label_text("This is a title"); //Set the title for the rectangle
        
        ascend::presenter::PresenterErrorCode error_code = ascend::presenter::SendMessage(channel, request)
        
        ```

    3.  Close Channel

        ```
        delete channel;
        ```


    如果需要发送一系列图片来展示视频的效果，则将[1](#zh-cn_topic_0147635264_li182791110135216)中的contentType改为ContentType::kVideo，并

    不断的调用SendMessage即可。

-   **修改源码**

    编译源码需要使用protoc编译proto文件，请从[https://github.com/protocolbuffers/protobuf/releases/](http://code.google.com/p/protobuf/downloads/list)中获取软件包protoc-3.5.1-linux-

    x86\_64.zip，并参照包中的readme进行安装。

    主要源码结构如下所示：

    ```
    common/presenter/agent                PresenterAgent源码根目录 
    ├─proto                             Protobuf消息定义 
    ├─include/ascendk/presenter/agent                          API头文件 
       ├─channel.h                      通用channel的接口，提供收发protobuf消息的功能
       ├─errors.h                       错误码
       ├─presenter_channel.h            封装了发送媒体数据到PresenterServer的功能 
       ├─presenter_types.h            封装了发送媒体数据到PresenterServer的功能 
    ├─src/asceddk/presenter/agent       源码目录 
       ├─channel                        与PresenterServer交互相关源码
          ├─default_channel.cpp         Channel类的默认实现，维护与Server间的长连接
          ├─default_channel.h           Channel类的默认实现的头文件
          ├─channel.cpp                 Channel类的默认实现的头文件
       ├─codec 
       ├─connection                     提供发送/接收protobuf消息的接口       
       ├─net                            网络连接相关源码, 完成收发字节数组的接口
          ├─socket.cpp                  Socket抽象类 
          ├─socket_factory.cpp          Socket工厂抽象类 
          ├─raw_socket.cpp              基于linux原生socket，不提供通道加密功能 
          ├─raw_socket_factory.cpp      RawSocket的工厂类 
       ├─presenter                      封装了发送媒体数据到PresenterServer的功能 
       ├─util                           工具类相关源码 
    ├─Makefile
    ```

    如果修改了presenter\_message.proto，则需要在proto文件夹下执行以下命令编译proto文件：

    protoc presenter\_message.proto --cpp\_out=./


