MyData = struct;
MyData.flag = uint8(170);
MyData.type = uint8(1);
MyData.value = uint16(6377);
MyData.position = single(1);
MyData.velocity = single(1);


data  = [MyData.flag MyData.type uint8(bi2de(bitget(MyData.value, 1:8))) uint8(bitshift(MyData.value, -8)) ...
    fliplr(uint8(hex2dec(reshape(num2hex(MyData.position),2,[])'))') fliplr(uint8(hex2dec(reshape(num2hex(MyData.velocity),2,[])'))')];
MyData.crc = crcccitt(data);

% fileID = fopen('myfile.bin','w');
% fwrite(fileID,MyData.flag,'uint8','ieee-le');
% fwrite(fileID,MyData.type,'uint8','ieee-le');
% fwrite(fileID,MyData.value,'uint16','ieee-le');
% fwrite(fileID,MyData.position,'single','ieee-le');
% fwrite(fileID,MyData.velocity,'single','ieee-le');
% fwrite(fileID,MyData.crc,'uint16','ieee-le');
% fclose(fileID);

s = serial('COM5');
set(s,'BaudRate',115200);
s.ByteOrder = 'littleEndian';
fopen(s);
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');
while (ishandle(H))
    fwrite(s,MyData.flag,'uint8','sync');
    fwrite(s,MyData.type,'uint8','sync');
    fwrite(s,MyData.value,'uint16','sync');
    fwrite(s,MyData.position,'single','sync');
    fwrite(s,MyData.velocity,'single','sync');
    fwrite(s,MyData.crc,'uint16','sync');
    pause(1);
end
fclose(s);
    
