/**
 * author levi
 * url http://levi.cg.am
 */
$(function() {
	var $parent = $('.keyword_box')
	$(document).mousemove(function(e) {
		if (!!this.move) {
			console.log(document.move_target.posix)
			var posix = !document.move_target ? {'x': 0, 'y': 0} : document.move_target.posix,
				callback = document.call_down || function() {
					
					$(this.move_target).css({
						'top': e.pageY - $parent.offset().top + $box.,
						'left': e.pageX - $parent.offset().left
					});
				};

			callback.call(this, e, posix);
		}
	}).mouseup(function(e) {
		if (!!this.move) {
			var callback = document.call_up || function(){};
			callback.call(this, e);
			$.extend(this, {
				'move': false,
				'move_target': null,
				'call_down': false,
				'call_up': false
			});
		}
	});

	var $box = $('#box').mousedown(function(e) {
		var offset = $(this).offset();
		console.log(e.pageX , $parent.offset().left,'-dsdsdss')
	    this.posix = {'x': e.pageX - $parent.offset().left, 'y': e.pageY - $parent.offset().top};
	    $.extend(document, {'move': true, 'move_target': this});
	}).on('mousedown', '#coor', function(e) {
	    var posix = {
	            'w': $box.width(), 
	            'h': $box.height(), 
	            'x': e.pageX, 
	            'y': e.pageY
	        };
	    
	    $.extend(document, {'move': true, 'call_down': function(e) {
	        $box.css({
	            'width': Math.max(30, e.pageX - posix.x + posix.w),
	            'height': Math.max(30, e.pageY - posix.y + posix.h)
	        });
	    }});
	    return false;
	});
});