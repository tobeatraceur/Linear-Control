function SerialFuc()
  delete(instrfindall)      % �رմ���
  global s;
  %s = serial('com3');       %ʹ��Ĭ�����ô�������s
  s = Bluetooth('BT04-A', 1)   %�������������ƣ�ͨ����
  fopen(s);                 %�򿪴���
  %set(s,'BytesAvailableFcnMode','Terminator'); %�����жϴ�����ʽ
  %set(s,'Terminator','H');
  %set(obj,'BytesAvailableFcnMode','byte'); 
  %set(obj,'BytesAvailableFcnCount', 240);
  %���ڼ�⵽���뻺���е�����240���ַ�����ʱ,���������жϡ�
  
  s.BytesAvailableFcn =@ReceiveCallback;       % �����ж���Ӧ��������

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
function ReceiveCallback( obj,event)     %�����ж���Ӧ����
   global s;
   a = fscanf(s)       % �������ݲ���ʾ���޷ֺţ�
   I = 'I received'    % �����Ƿ��ж���Ӧ�����������������޷ֺţ�
end
