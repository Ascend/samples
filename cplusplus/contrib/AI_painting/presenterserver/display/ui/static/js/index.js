var valHtml = function (str) {
    return '<div class="box-item ' + str + '" data-id="' + str + '" id="box' + str + '">' +
        '<i class="close"></i></div>'
}
var selectHtmlList = {}
var sendData = {}
var selectName = []
var aiImg = new Array(3)
var posterImg = ''
var next_lock = false

var nowSwiper = 'slide1'

var mySwiper = new Swiper('.swiper-container', {
    // autoplay: true,//可选选项，自动滑动
    navigation: {
        nextEl: '.swiper-button-next',
        prevEl: '.swiper-button-prev',
    },
    spaceBetween: 10,
    init: false,
    on: {
        slideChangeTransitionEnd: function () {
            nowSwiper = 'slide' + (this.activeIndex + 1)
            $('#' + nowSwiper).find('.cont').text(nameval)
            $('#' + nowSwiper).find('.name').text(isval)
        }
    }
})

// 触发打印
function doPrint() {
    $('body').css({ 'width': "100%" })
    Print('#imgPoster', {
        onStart: function () {
            // console.log('onStart', new Date())
        },
        onEnd: function () {
            // console.log('onEnd', new Date())
            $('body').css({ 'width': "1920px", 'height': '1080px' })
        }
    })
}

var photo_ch_W = $('.photo_ch').width(),
    photo_ch_H = $('.photo_ch').height()
function initBox(list) {
    let $box = $("#containment-wrapper")
    if (!$box.find('.bg')) {
        $box.append('<img src="./img/img1.png" alt="" class="bg">')
    }
    list.forEach(function (res) {
        if (selectHtmlList[res]) {
            return
        }
        selectHtmlList[res] = {}
        sendData[res] = [{}]
        selectHtmlList[res].html = $(valHtml(res)).appendTo($box)
        var doc = selectHtmlList[res].html
        // setTimeout(() => {
        selectHtmlList[res].size = (9 * (doc.width() * doc.height()) / (photo_ch_W * photo_ch_H))
        selectHtmlList[res].left = doc.position().left
        selectHtmlList[res].top = doc.position().top
        sendData[res][0]['size'] = (9 * (doc.width() * doc.height()) / (photo_ch_W * photo_ch_H))
        sendData[res][0]['coor_x'] = (doc.position().left + (doc.width() / 2)) / photo_ch_W
        sendData[res][0]['coor_y'] = (doc.position().top + (doc.height() / 2)) / photo_ch_H
        // }, 250);



        selectHtmlList[res].html.resizable({
            containment: "#containment-wrapper",
            stop: function (e) {
                let position = $(e.target).position()
                // console.log('缩放', res)
                selectHtmlList[res].size = (9 * (e.target.clientWidth * e.target.clientHeight) / (photo_ch_W * photo_ch_H))
                selectHtmlList[res].isOld = true
                sendData[res][0]['coor_x'] = (position.left + (e.target.clientWidth / 2)) / photo_ch_W
                sendData[res][0]['coor_y'] = (position.top + (e.target.clientHeight / 2)) / photo_ch_H
                sendData[res][0]['size'] = (9 * (e.target.clientWidth * e.target.clientHeight) / (photo_ch_W * photo_ch_H))
                getImg(selectHtmlList)
            }
        }).draggable({
            containment: "#containment-wrapper",
            scroll: false,
            stop: function (e) {
                // console.log('拖动', res)
                let position = $(e.target).position()
                selectHtmlList[res].left = position.left
                selectHtmlList[res].top = position.top
                sendData[res][0]['coor_x'] = (position.left + (e.target.clientWidth / 2)) / photo_ch_W
                sendData[res][0]['coor_y'] = (position.top + (e.target.clientHeight / 2)) / photo_ch_H
                selectHtmlList[res].isOld = true
                getImg(selectHtmlList)
            }
        })
    })

    Object.keys(selectHtmlList).forEach(function (res) {
        if (list.indexOf(res) === -1) {
            delete selectHtmlList[res]
            delete sendData[res]
            $("#box" + res).remove()
        }
    })

    setTimeout(() => {
        Object.keys(selectHtmlList).forEach((res) => {
            var item = selectHtmlList[res]
            if (item.isOld) return
            item.html.css({ "position": "absolute", "left": item.left, "top": item.top + 20, "margin-top": 0, "margin-right": 0 })
        })
    }, 500);
    getImg(selectHtmlList)
}
$(document).on('click', '.close', function () {
    let id = $(this).parent().attr('data-id')
    $(this).parent().remove()
    $('.select-' + id).remove()
    $("[value ='" + id + "']").attr("checked", false)
    delete selectHtmlList[id]
    delete sendData[id]
    ws.send('next;layout:' + JSON.stringify(sendData));
    return
})
function getImg(selectHtmlList) {
    next_lock = true
    // console.log('请求中')
    if (Object.keys(selectHtmlList).length < 3) {
        return
    }
    // setTimeout(() => {
    ws.send('next;layout:' + JSON.stringify(sendData));
    return new Promise((rej) => {
        /* selectHtmlList {Object}
         **  示例 {grass: {html: init(1), width: 148, height: 78, left: 157, top: 0}}
         **  grass: key名即选择的元素名 width 宽 height 高 left 距离左边距离 top 距离顶部距离
         */
        // $.ajax({
        //     url: '',
        //     data: selectHtmlList,
        //     success: function (res) {
        //         /* rej() */
        //     }
        // })
        // var res = ['https://ss1.bdstatic.com/70cFuXSh_Q1YnxGkpoWK1HF6hhy/it/u=3073032892,1654551905&fm=26&gp=0.jpg', 'https://ss0.bdstatic.com/70cFuHSh_Q1YnxGkpoWK1HF6hhy/it/u=2374971164,3821020068&fm=26&gp=0.jpg']
        // $('.photo_a img').attr('src', res[0])
        // $('.photo_b img').attr('src', res[1])
        // $('.swiper-slide').find('.img3').attr('src', res[1])
        // $('.swiper-slide').find('.img2').attr('src', res[0])
        // aiImg = [res[0], res[1]]
        // console.log('请求结束')
        rej()
        next_lock = false
    })
    // }, 1000);


}

function imgLoad(el) {
    return new Promise(function (resolve) {
        var imgList = el.find("img").not('.poster');
        var i = 0;
        var length = imgList.length;
        var per = 0;
        var imgsNum = $("img").length;
        var i = 0;
        imgList.each(function () {
            var img = new Image();
            img.src = $(this).attr("src");

            img.onload = function () {
                i++;
                per = parseInt((i / length) * 100, 10);

                if (per == 100) {
                    resolve();
                }
            };
        });
    });
}

function createImg() {
    let $slide1 = $('#' + nowSwiper)
    $('.poster').attr('src', '')
    return new Promise((rej) => {
        imgLoad($slide1).then(function () {
            let doc = document.getElementById(nowSwiper)
            document.documentElement.scrollTo(0, 0)
            var canvas = document.createElement('canvas');
            canvas.width = 1748;
            canvas.height = 1401;
            // canvas.height = 1181;
            canvas.style.width = '1748px';
            canvas.style.height = '1401px';
            // canvas.style.height = '1181px';
            var context = canvas.getContext('2d');
            context.scale(2.774, 2.811);
            new html2canvas(doc, {
                canvas: canvas,
                useCORS: true,
                logging: true,
                backgroundColor: null,
                width: 1748,
            }).then(canvas => {
                var src = canvas.toDataURL('image/png')
                var url
                url = canvas.toDataURL('image/png')

                canvas.toBlob((url) => {
                    url = URL.createObjectURL(url)
                    $('.poster').attr('src', url)
                    upImg(url).base().then(() => {
                        rej()
                    })
                })

                // $('.poster').attr('src', url).show()
            })
        })
    })
}

function upImg(src) {
    return {
        base: function () {
            return new Promise((rej) => {
                // $.ajax({
                //     url: '',
                //     data: {
                //         src: src
                //     },
                //     success: function () {
                // rej()
                //     }
                // })
                // console.log('上传base', src)
                rej()
            })
        },
        formData: function () {
            return new Promise((rej) => {
                const formData = new FormData()
                formData.append('base64', src)
                // $.ajax({
                //     url: '',
                //     data: formData,
                //     success: function () {
                //      rej()
                //     }
                // })
                // console.log('上传formData', formData)
                rej()
            })
        }
    }
}

function createTextImg() {
    return new Promise((rej) => {
        document.documentElement.scrollTo(0, 0)
        new html2canvas(document.getElementById('containment-wrapper')).then(canvas => {
            aiImg[3] = canvas.toDataURL('image/png')
            rej()
        })
    })
}

var nameval = '';
var isval = '';
$(function () {

    $("body").keyup(function (e) {
        let focusDom = document.activeElement.tagName
        if ((e.key.toLocaleLowerCase() === 'q') && focusDom === 'BODY') {
            window.location.reload()
        }
    })

    var is_sel = false
    $(".drop_down_box").click(function () {
        if (is_sel == false) {
            is_sel = true
            $(".select_group").show()
        } else {
            is_sel = false
            $(".select_group").hide()
        }
    })
    // 选择物体
    $(".select_btn").click(function () {
        var selecttext = ''
        selectName = []
        $('input:checkbox[name="multiple"]:checked').each(function () {
            var id = $(this).val()
            selectName.push(id)
            selecttext += '<div class="selecttext select-' + id + '">' + id + ';</div>'
        });
        if (selectName.length < 3) {
            alert('请选择至少三个对象')
            return
        } else if (selectName.length > 9) {
            alert('请选择少于10个对象')
            return
        }
        $(".is_select").html(selecttext)
        is_sel = false
        $(".select_group").hide()
        initBox(selectName)
    })
    // 返回上一页
    $(".back").click(function () {
        $('.back').hide()
        $(".container").show()
        $(".create_container").hide()
        $('.Postcard_text').val('')
        $('.Sign_text').val('')
        $('.text-box').find('p').text('')

        $(".box-item .close").css('display', 'block')
        $(".box-item").css("border", "1px solid #fff")
        nameval = ''
        isval = ''

        mySwiper.slideTo(0)
    })
    // 进入样式选择页
    $(".next_btn").click(function () {
        $(".box-item .close").css('display', 'none')
        $(".box-item").css("border", "none")
        $('.back').show()
        if (next_lock) {
            return
        }
        if (Object.keys(selectHtmlList).length < 3) {
            alert('请至少选择三个对象')
            return
        }
        createTextImg().then((res) => {
            $(".container").hide()
            $(".create_container").show()
            $(".swiper-slide").find('.img1').attr('src', aiImg[3])
            mySwiper.init(); //现在才初始化
        })
    })
    // 点击打印
    $(".print_btn").click(function () {
        var Postcard_text = $('#' + nowSwiper).find('.cont').text();
        var Sign_text = $('#' + nowSwiper).find('.name').text();
        if (Postcard_text == '') {
            alert('请输入明信片内容')
            return
        }
        if (Sign_text == '') {
            alert('请添加署名')
            return
        }
        $(".Printing").show()
        setTimeout(() => {
            createImg().then(() => {
                $(".Printing").hide()
                // $(".finish").show()
                // console.log('开始打印')
                doPrint()
            })
        }, 1000);
        // setTimeout(() => {
        //     createImg().then( () => {
        //         // $(".Printing").hide()
        //         $(".finish").hide()
        //     })
        // }, 3000);
    })

    $('.Postcard_text').on('input propertychange', function () {
        nameval = $('.Postcard_text').val()
        if (nameval.length > 27) {
            return
        }
        $('.cont').text(nameval)
    })
    $('.Sign_text').on('input propertychange', function () {
        isval = $('.Sign_text').val()
        if (isval.length > 27) {
            return
        }
        $('.name').text(isval)
    })
    $(".Postcard_btn").click(function () {
        let val = $('.Postcard_text').val()
        if (val.length < 5 || val.length > 15) {
            alert('内容只能在5-15个字符内')
            return
        }
        $('.cont').text(val)
    })
    $(".Sign_btn").click(function () {
        let val = $('.Sign_text').val()
        if (val.length < 1 || val.length > 7) {
            alert('署名只能在1-7个字符内')
            return
        }
        $('.name').text(val)
    })

    $(".box-img").click(function () {
        $(this).hide()
    })
})
// 数据交互
let upDataStyle = 0;
var canvas_a = document.getElementById("canvas_a"),
    canvas_b = document.getElementById("canvas_b")
var ctx_a = canvas_a.getContext("2d"),
    ctx_b = canvas_b.getContext("2d")
$('#fpswapper').hide()
$('.loading').hide()
var ws = new WebSocket(wsUrl);
var onmessageflag = false;
var src_a = '', src_b = ''
function startViewVideo() {
    $('.loading').show()
    $('#canvas_a').hide()
    ws.onopen = function () {
    };
    var count = 0;
    var timestart = 0;
    // 接受websocket回传数据
    ws.onmessage = function (evt) {
        $('.loading').hide()
        $('#canvas_a').show()
        var data = JSON.parse(evt.data)
        var rectangles = []
        if (data['status'] == 'ok') {
            $('#fpsval').text(data.fps);
            $('.loading').hide();
            // $('#load_media').show();
            if (data.view == 'ColorMap') {
                src_a = "data:image/jpeg;base64," + data['image'];
            } else if (data.view == 'ResultImage') {
                src_b = "data:image/jpeg;base64," + data['image'];
            }
            // $('#load_media').attr('src', src);
            if (data['type'] == 'video') {
                // $('#fpswapper').show();  // comment fps show
                rectangles = data['rectangle_list']
            }
            var wantedWidth = 464
            var img_a = new Image()
            // 获取回传图片渲染到canvas
            // img_a.src = src
            // 左图
            img_a.src = src_a
            img_a.onload = function () {
                scale_factor = wantedWidth / img_a.width
                canvas_a.setAttribute("width", wantedWidth)
                canvas_a.setAttribute("height", img_a.height * scale_factor)
                ctx_a.drawImage(img_a, 0, 0, wantedWidth, img_a.height * scale_factor)
                for (var index in rectangles) {
                    var pos = rectangles[index].slice(0, 4)  //
                    for (var i in pos) {
                        pos[i] = pos[i] * scale_factor
                    }
                    var msg = rectangles[index].slice(4, 5)
                    //add space between msg and face
                    //if upper space is not enough show the msg at the bottom
                    if (50 > pos[1]) {
                        ctx_a.fillText(msg, pos[0], pos[3] + 50)
                    }
                    else {
                        ctx_a.fillText(msg, pos[0], pos[1] - 10)
                    }
                    ctx_a.beginPath()
                    // 1/3 space draw line
                    ctx_a.moveTo(pos[0], pos[1])
                    ctx_a.lineTo(pos[0], pos[3] / 3 + pos[1] * 2 / 3)
                    ctx_a.moveTo(pos[0], pos[3] * 2 / 3 + pos[1] / 3)
                    ctx_a.lineTo(pos[0], pos[3])
                    ctx_a.lineTo(pos[0] * 2 / 3 + pos[2] / 3, pos[3])
                    ctx_a.moveTo(pos[0] / 3 + pos[2] * 2 / 3, pos[3])
                    ctx_a.lineTo(pos[2], pos[3])
                    ctx_a.lineTo(pos[2], pos[3] * 2 / 3 + pos[1] / 3)
                    ctx_a.moveTo(pos[2], pos[3] / 3 + pos[1] * 2 / 3)
                    ctx_a.lineTo(pos[2], pos[1])
                    ctx_a.lineTo(pos[2] * 2 / 3 + pos[0] / 3, pos[1])
                    ctx_a.moveTo(pos[0] * 2 / 3 + pos[2] / 3, pos[1])
                    ctx_a.lineTo(pos[0], pos[1])
                    ctx_a.stroke()
                }
            }
            // 右图
            var img_b = new Image()
            img_b.src = src_b
            img_b.onload = function () {
                scale_factor = wantedWidth / img_b.width
                canvas_b.setAttribute("width", wantedWidth)
                canvas_b.setAttribute("height", img_b.height * scale_factor)
                ctx_b.drawImage(img_b, 0, 0, wantedWidth, img_b.height * scale_factor)
                for (var index in rectangles) {
                    var pos = rectangles[index].slice(0, 4)  //
                    for (var i in pos) {
                        pos[i] = pos[i] * scale_factor
                    }
                    var msg = rectangles[index].slice(4, 5)
                    //add space between msg and face
                    //if upper space is not enough show the msg at the bottom
                    if (50 > pos[1]) {
                        ctx_b.fillText(msg, pos[0], pos[3] + 50)
                    }
                    else {
                        ctx_b.fillText(msg, pos[0], pos[1] - 10)
                    }
                    ctx_b.beginPath()
                    // 1/3 space draw line
                    ctx_b.moveTo(pos[0], pos[1])
                    ctx_b.lineTo(pos[0], pos[3] / 3 + pos[1] * 2 / 3)
                    ctx_b.moveTo(pos[0], pos[3] * 2 / 3 + pos[1] / 3)
                    ctx_b.lineTo(pos[0], pos[3])
                    ctx_b.lineTo(pos[0] * 2 / 3 + pos[2] / 3, pos[3])
                    ctx_b.moveTo(pos[0] / 3 + pos[2] * 2 / 3, pos[3])
                    ctx_b.lineTo(pos[2], pos[3])
                    ctx_b.lineTo(pos[2], pos[3] * 2 / 3 + pos[1] / 3)
                    ctx_b.moveTo(pos[2], pos[3] / 3 + pos[1] * 2 / 3)
                    ctx_b.lineTo(pos[2], pos[1])
                    ctx_b.lineTo(pos[2] * 2 / 3 + pos[0] / 3, pos[1])
                    ctx_b.moveTo(pos[0] * 2 / 3 + pos[2] / 3, pos[1])
                    ctx_b.lineTo(pos[0], pos[1])
                    ctx_b.stroke()
                }
            }
            // 图片传递到样式选择页，用作截图打印
            $('.swiper-slide').find('.img2').attr('src', img_a.src)
            $('.swiper-slide').find('.img3').attr('src', img_b.src)
        }
    }
}
startViewVideo();
