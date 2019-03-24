ReceiveData = struct;
ReceiveData.flag = uint8(170);
ReceiveData.type = uint8(1);
ReceiveData.value = int16(6377);
ReceiveData.position = int32(1);
ReceiveData.velocity = int32(1);

data  = [ReceiveData.flag ...   % uint8
    ReceiveData.type ...        % uint8
    uint8(bi2de(bitget(ReceiveData.value, 1:8))) uint8(bi2de(bitget(ReceiveData.value, 9:16))) ...     % uint16 to uint8     pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.position, 1:8))) uint8(bi2de(bitget(ReceiveData.position, 9:16)))  ...              % int32 to uint8      pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.position, 17:24))) uint8(bi2de(bitget(ReceiveData.position, 25:32)))  ...              % int32 to uint8      pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.velocity, 1:8))) uint8(bi2de(bitget(ReceiveData.velocity, 9:16)))  ...              % int32 to uint8      pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.velocity, 17:24))) uint8(bi2de(bitget(ReceiveData.velocity, 25:32))) ];              % int32 to uint8      pay attention to endian order];                % int32 to uint8      pay attention to endian order

ReceiveData.crc = crcccitt(data);

% fileID = fopen('myfile.bin','w');
% fwrite(fileID,ReceiveData.flag,'uint8','ieee-le');
% fwrite(fileID,ReceiveData.type,'uint8','ieee-le');
% fwrite(fileID,ReceiveData.value,'int16','ieee-le');
% fwrite(fileID,ReceiveData.position,'int32','ieee-le');
% fwrite(fileID,ReceiveData.velocity,'int32','ieee-le');
% fwrite(fileID,ReceiveData.crc,'uint16','ieee-le');
% fclose(fileID);

s = serial('COM5');
set(s,'BaudRate',115200);
s.ByteOrder = 'littleEndian';
fopen(s);
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');
                
fwrite(s,ReceiveData.flag,'uint8','sync');
fwrite(s,ReceiveData.type,'uint8','sync');
fwrite(s,ReceiveData.value,'int16','sync');
fwrite(s,ReceiveData.position,'int32','sync');
fwrite(s,ReceiveData.velocity,'int32','sync');
fwrite(s,ReceiveData.crc,'uint16','sync');

while (ishandle(H))

    
    A = fread(s,1,'uchar');
    if (A(1) == uint8(170)) % the head of a frame 
        B = fread(s,13,'uchar');
%         dec2hex([A;B])

        crc = B(12:13);
        crc16 = crcccitt([A(1);B(1:11)]);
        my_crc = [uint8(bi2de(bitget(crc16, 1:8)));uint8(bitshift(crc16, -8))];        % pay attention to the order because of little endian
        
        if (sum(my_crc == crc) == 2)
            disp('get a frame of data')
            ReceiveData.flag = A(1);
            ReceiveData.type = B(1);    
            ReceiveData.value = typecast(uint8(fliplr(B(2:3))), 'int16');               % flip because of little endian
            ReceiveData.position = typecast(uint8(fliplr(B(4:7))), 'int32');            % flip because of little endian
            ReceiveData.velocity = typecast(uint8(fliplr(B(8:11))), 'int32');           % flip because of little endian
            ReceiveData.crc = my_crc;
            ReceiveData
            
            % for test!!!!!!!!!!!!!
            ReceiveData.velocity = ReceiveData.velocity - 1;
            
            data  = [ReceiveData.flag ...   % uint8
                ReceiveData.type ...        % uint8
                uint8(bi2de(bitget(ReceiveData.value, 1:8))) uint8(bi2de(bitget(ReceiveData.value, 9:16))) ...     % uint16 to uint8     pay attention to endian order
                uint8(bi2de(bitget(ReceiveData.position, 1:8))) uint8(bi2de(bitget(ReceiveData.position, 9:16)))  ...              % int32 to uint8      pay attention to endian order
                uint8(bi2de(bitget(ReceiveData.position, 17:24))) uint8(bi2de(bitget(ReceiveData.position, 25:32)))  ...              % int32 to uint8      pay attention to endian order
                uint8(bi2de(bitget(ReceiveData.velocity, 1:8))) uint8(bi2de(bitget(ReceiveData.velocity, 9:16)))  ...              % int32 to uint8      pay attention to endian order
                uint8(bi2de(bitget(ReceiveData.velocity, 17:24))) uint8(bi2de(bitget(ReceiveData.velocity, 25:32))) ];              % int32 to uint8      pay attention to endian order];                % int32 to uint8      pay attention to endian order

            ReceiveData.crc = crcccitt(data);
            
            fwrite(s,ReceiveData.flag,'uint8','sync');
            fwrite(s,ReceiveData.type,'uint8','sync');
            fwrite(s,ReceiveData.value,'int16','sync');
            fwrite(s,ReceiveData.position,'int32','sync');
            fwrite(s,ReceiveData.velocity,'int32','sync');
            fwrite(s,ReceiveData.crc,'uint16','sync');
        end

    end
    
    pause(1);
end
fclose(s);
    
