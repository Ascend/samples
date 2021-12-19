;(function($,window,document,undefined){var Drag=function(ele,opt){this.$ele=ele,this.x=0,this.y=0,this.defaults={parent:'parent',randomPosition:true,direction:'all',handler:false,dragStart:function(x,y){},dragEnd:function(x,y){},dragMove:function(x,y){}},this.options=$.extend({},this.defaults,opt)}
Drag.prototype={run:function(){var $this=this;var element=this.$ele;var randomPosition=this.options.randomPosition;var direction=this.options.direction;var handler=this.options.handler;var parent=this.options.parent;var isDown=false;var fun=this.options;var X=0,Y=0,moveX,moveY;element.find('*').not('img').mousedown(function(e){e.stopPropagation();});if(parent=='parent'){parent=element.parent();}else{parent=element.parents(parent);}
if(!handler){handler=element;}else{handler=element.find(handler);}
parent.css({position:'relative'});element.css({position:'absolute'});var boxWidth=0,boxHeight=0,sonWidth=0,sonHeight=0;initSize();if(randomPosition){randomPlace();}
$(window).resize(function(){initSize();if(randomPosition){randomPlace();}});function initSize(){boxWidth=parent.outerWidth();boxHeight=parent.outerHeight();sonWidth=element.outerWidth();sonHeight=element.outerHeight();}
function randomPlace(){if(randomPosition){var randX=parseInt(Math.random()*(boxWidth-sonWidth));var randY=parseInt(Math.random()*(boxHeight-sonHeight));if(direction.toLowerCase()=='x'){element.css({left:randX});}else if(direction.toLowerCase()=='y'){element.css({top:randY});}else{element.css({left:randX,top:randY});}}}
handler.css({cursor:'move'}).mousedown(function(e){isDown=true;X=e.pageX;Y=e.pageY;$this.x=element.position().left;$this.y=element.position().top;element.addClass('on');fun.dragStart(parseInt(element.css('left')),parseInt(element.css('top')));return false;});$(document).mouseup(function(e){fun.dragEnd(parseInt(element.css('left')),parseInt(element.css('top')));element.removeClass('on');isDown=false;});$(document).mousemove(function(e){moveX=$this.x+e.pageX-X;moveY=$this.y+e.pageY-Y;function thisXMove(){if(isDown==true){element.css({left:moveX});}else{return;}
if(moveX<0){element.css({left:0});}
if(moveX>(boxWidth-sonWidth)){element.css({left:boxWidth-sonWidth});}
return moveX;}
function thisYMove(){if(isDown==true){element.css({top:moveY});}else{return;}
if(moveY<0){element.css({top:0});}
if(moveY>(boxHeight-sonHeight)){element.css({top:boxHeight-sonHeight});}
return moveY;}
function thisAllMove(){if(isDown==true){element.css({left:moveX,top:moveY});}else{return;}
if(moveX<0){element.css({left:0});}
if(moveX>(boxWidth-sonWidth)){element.css({left:boxWidth-sonWidth});}
if(moveY<0){element.css({top:0});}
if(moveY>(boxHeight-sonHeight)){element.css({top:boxHeight-sonHeight});}}
if(isDown){fun.dragMove(parseInt(element.css('left')),parseInt(element.css('top')));}else{return false;}
if(direction.toLowerCase()=="x"){thisXMove();}else if(direction.toLowerCase()=="y"){thisYMove();}else{thisAllMove();}});}}
$.fn.myDrag=function(options){var drag=new Drag(this,options);drag.run();return this;}})(jQuery,window,document);