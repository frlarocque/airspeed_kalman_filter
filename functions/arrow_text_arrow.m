function arrow_text_arrow(p1_x,p2_x,y,le,string,line_width,font_size)

%font_size = 13;
%line_width = 2;
%string = 'Growth';
%p1_x = 10;
%p2_x = 15;
%le = 1.5;
%y = 1;

offset = ((p2_x-p1_x)-le)./2;

[xt_1, yt_1] = ds2nfu(p1_x, y);
[xh_1, yh_1] = ds2nfu(p1_x+offset, y);
[xt_2, yt_2] = ds2nfu(p2_x, y);
[xh_2, yh_2] = ds2nfu(p2_x-offset, y);

annotation('textarrow',[xh_2 xt_2],[yh_2 yt_2],'String',string,'FontSize',font_size,'Linewidth',line_width)
annotation('textarrow',[xh_1 xt_1],[yh_1 yt_1],'String','','FontSize',font_size,'Linewidth',line_width)

end