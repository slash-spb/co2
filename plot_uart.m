f = fopen('/dev/ttyUSB0','r');

len = 3000;

c = zeros(len,1);
i=1;
h = plot(c);
axis([0 len -30 30]);
while true
  z = fgetl(f); 
  if( ~isempty(isdigit(z)) )
      
    c(i)=str2num(z);
    i=i+1;
        
    if ( i==len )
      i=1;
      set(h,'YData',c(:,1));
      drawnow;
    end
  end
end