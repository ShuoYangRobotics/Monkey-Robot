ReceiveData = struct;

s = serial('COM5');
set(s,'BaudRate',115200);
s.ByteOrder = 'littleEndian';
fopen(s);
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');
while (ishandle(H))
    A = fread(s,1,'uchar');
    if (A(1) == uint8(170)) % the head of a frame 
        B = fread(s,13,'uchar');
        
        crc = B(12:13);
        crc16 = crcccitt([A(1);B(1:11)]);
        my_crc = [uint8(bi2de(bitget(crc16, 1:8)));uint8(bitshift(crc16, -8))];
        
        if (sum(my_crc == crc) == 2)
            disp('get a frame of data')
            ReceiveData.flag = A(1);
            ReceiveData.type = B(1);
            ReceiveData.value = typecast(uint8(fliplr(B(2:3))), 'uint16');
            ReceiveData.position = typecast(uint8(fliplr(B(4:7))), 'single');
            ReceiveData.velocity = typecast(uint8(fliplr(B(8:11))), 'single');
            ReceiveData
        end

    end
    pause(0.5);
end
fclose(s);