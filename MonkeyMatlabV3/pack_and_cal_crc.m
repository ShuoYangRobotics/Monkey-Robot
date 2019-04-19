function crc = pack_and_cal_crc(ReceiveData)

data  = [ReceiveData.flag ...   % uint8
    ReceiveData.type ...        % uint8
    uint8(bi2de(bitget(ReceiveData.value, 1:8))) uint8(bi2de(bitget(ReceiveData.value, 9:16))) ...     % uint16 to uint8     pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.position, 1:8))) uint8(bi2de(bitget(ReceiveData.position, 9:16)))  ...              % int32 to uint8      pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.position, 17:24))) uint8(bi2de(bitget(ReceiveData.position, 25:32)))  ...              % int32 to uint8      pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.velocity, 1:8))) uint8(bi2de(bitget(ReceiveData.velocity, 9:16)))  ...              % int32 to uint8      pay attention to endian order
    uint8(bi2de(bitget(ReceiveData.velocity, 17:24))) uint8(bi2de(bitget(ReceiveData.velocity, 25:32))) ];              % int32 to uint8      pay attention to endian order];                % int32 to uint8      pay attention to endian order

crc = crcccitt(data);

end