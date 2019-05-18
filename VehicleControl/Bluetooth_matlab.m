function Bluetooth_matlab()
  delete(instrfindall)      % 关闭串口
  global s;
  %s = serial('com3');       %使用默认设置创建串口s
  s = Bluetooth('BT04-A', 1)   %连接蓝牙（名称，通道）
  fopen(s);                 %打开串口
  %set(s,'BytesAvailableFcnMode','Terminator'); %设置中断触发方式
  %set(s,'Terminator','H');
  %set(obj,'BytesAvailableFcnMode','byte'); 
  %set(obj,'BytesAvailableFcnCount', 240);
  %串口检测到输入缓存中到达了240个字符数据时,触发串口中断。
  
  s.BytesAvailableFcn =@ReceiveCallback;       % 定义中断响应函数对象


while(1)
    x=input('input:','s');
    command = string(x);
    disp('The commnd is:');disp(command);
    fprintf(s,command);
    if(command=='gxdsb')
        break;
    end
end
end
function ReceiveCallback( obj,event)     %创建中断响应函数
   global s;
   a = fscanf(s)       % 接收数据并显示（无分号）
   I = 'I received'    % 检验是否中断响应函数正常被触发（无分号）
end
