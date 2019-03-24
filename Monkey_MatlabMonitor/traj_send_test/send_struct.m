function send_struct(s, SendData)
    fwrite(s,SendData.flag,'uint8','sync');
    fwrite(s,SendData.type,'uint8','sync');
    fwrite(s,SendData.value,'int16','sync');
    fwrite(s,SendData.position,'int32','sync');
    fwrite(s,SendData.velocity,'int32','sync');
    fwrite(s,SendData.crc,'uint16','sync');
end