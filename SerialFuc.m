function SerialFuc()
  delete(instrfindall)      % �رմ���
  global s;
  %s = serial('com3');       %ʹ��Ĭ�����ô�������s
  s = Bluetooth('1234', 5)   %�������������ƣ�ͨ����
  fopen(s);                 %�򿪴���
  set(s,'BytesAvailableFcnMode','Terminator'); %�����жϴ�����ʽ
  set(s,'Terminator','H');
  %set(obj,'BytesAvailableFcnMode','byte'); 
  %set(obj,'BytesAvailableFcnCount', 240);
  %���ڼ�⵽���뻺���е�����240���ַ�����ʱ,���������жϡ�
  
  s.BytesAvailableFcn =@ReceiveCallback;       % �����ж���Ӧ��������
  for k=1:1:5               % ÿ���뷢���ַ�����ѭ�����
      fprintf(s,'test');
      pause(2);
  end    
end
function ReceiveCallback( obj,event)     %�����ж���Ӧ����
   global s;
   a = fscanf(s)       % �������ݲ���ʾ���޷ֺţ�
   I = 'I received'    % �����Ƿ��ж���Ӧ�����������������޷ֺţ�
end
