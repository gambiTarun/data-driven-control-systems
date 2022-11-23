av_Cold_Flow = zeros(6000,1);

for t=1:40
    
    if t==1
        av_Cold_Flow(1:t*150-1) = mean(Cold_Flow(1:t*150-1));
    else
        av_Cold_Flow(((t-1)*150):t*150-1) = mean(Cold_Flow(((t-1)*150):t*150-1));
    end
end

x = downsample(u_Cold(1:6001),150);
y = downsample(av_Cold_Flow(1:6000),150);

p1 = polyfit(x(1:21),y(1:21),3);
p2 = polyfit(x(22:40),y(22:40),5);

x1 = 20:1:40;
y1 = polyval(p1,x1);

x2 = 40:-1:22;
y2 = polyval(p2,x2);

figure
plot(x(1:21),y(1:21),'o',x1,y1)
figure
plot(x(22:40),y(22:40),'o',x2,y2)