% this script can only be executed after: 
%      1. script_Monkey_proto_iLQR is called. And new_x_list,new_u_list are
%         obtained
%      2. matlab can access to COM5, and COM5 connects to bluetooth to
%         monkey robot

% this script wraps new_x_list(2) ( pos for left shoulder motor)
%                   new_x_list(5) ( vel for left shoulder motor)
%                   new_x_list(3) ( pos for right shoulder motor)
%                   new_x_list(6) ( vel for right shoulder motor)
% and then send through serial port


% different trajectory for different start and end distance
% 041to041 means that start distance is 0.41cm and end distance is 0.45cm
load('test045_to_045.mat');
% load('test_traj_041to045.mat');
%% prepare serial port
s = serial('COM5');
set(s,'BaudRate',115200);
s.ByteOrder = 'littleEndian';
fopen(s);

%% step 1
SendData = struct;
% first config SendData to be a notification message
SendData.flag = uint8(170);
SendData.type = uint8(2);    % type = 2 means this message notify monkey to prepare to receive trajectory
SendData.value = int16(0);
SendData.position = int32(0);
SendData.velocity = int32(0);
SendData.crc = pack_and_cal_crc(SendData);

send_struct(s, SendData);

% a GUI to break the loop
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');
send_on = 1;                
send_left_or_right = 1;     % if 1, send left trajectory, if 2, send right trajectory
traj_idx = 1;
FSM_state = 1;  % this sender is a finite state machine. 
                % state = 1: wait for the ack of the notification package 
                % state = 2: send a trajectory package
                % state = 3: wait for the ack of the previous trajectory package
                % state = 4: a notification message to start trajectory
ack_wait_times = 0;
databuffer_wait = 0;
MAX_ACK_WAIT_TIMES = 10;   % if ack_wait_times
ReceiveData = struct;                
while (ishandle(H) && send_on)

    switch FSM_state
        case 1
            while (s.BytesAvailable == 0)
                disp('no data in serial buffer')
                databuffer_wait  = databuffer_wait + 1
                if (databuffer_wait > 100)
                    send_on = 0;
                    break;
                end
            end
            if (send_on == 0)
                break;
            end
            % wait for the ack of the notification package 
            A = fread(s,1,'uchar');
            if (A(1) == uint8(170)) % the head of a frame 
                B = fread(s,13,'uchar');
                crc = B(12:13);
                crc16 = crcccitt([A(1);B(1:11)]);
                my_crc = [uint8(bi2de(bitget(crc16, 1:8)));uint8(bitshift(crc16, -8))];        % pay attention to the order because of little endian

                if (sum(my_crc == crc) == 2)
                    ReceiveData.type = B(1);  
                    if (ReceiveData.type == 40)  %  ACK 
                        
                        if (send_left_or_right == 1)
                            disp('got ACK from MCU, ready to upload left trajectory point 1')
                        else
                            
                            disp('got ACK from MCU, ready to upload right trajectory point 1')
                        end
                        FSM_state = 2;
                        continue;
                    end
                end
            end
            
            ack_wait_times = ack_wait_times + 1;
            if (ack_wait_times > MAX_ACK_WAIT_TIMES)
                disp('no ACK!!!! something is wrong')
            	break;
            end
            
        case 2
            traj_idx
            SendData.flag = uint8(170);
            if (send_left_or_right == 1)
                SendData.type = uint8(3);    % type = 3 means this message contains a left trajectory point
                SendData.value = int16(traj_idx-1);
                SendData.position = int32(new_x_list(2,traj_idx)*1000);
                SendData.velocity = int32(new_x_list(5,traj_idx)*1000);
            else
                SendData.type = uint8(4);    % type = 4 means this message contains a right trajectory point
                SendData.value = int16(traj_idx-1);
                SendData.position = int32(new_x_list(3,traj_idx)*1000);
                SendData.velocity = int32(new_x_list(6,traj_idx)*1000);
            end
            SendData.crc = pack_and_cal_crc(SendData);
            send_struct(s, SendData);
            
            FSM_state = 3;
            ack_wait_times = 0;
        case 3
            while (s.BytesAvailable == 0)
                disp('no data in serial buffer')
            end
            s.BytesAvailable
            % wait for the ack of the notification package 
            A = fread(s,1,'uchar');
            if (A(1) == uint8(170)) % the head of a frame 
                B = fread(s,13,'uchar');
                crc = B(12:13);
                crc16 = crcccitt([A(1);B(1:11)]);
                my_crc = [uint8(bi2de(bitget(crc16, 1:8)));uint8(bitshift(crc16, -8))];        % pay attention to the order because of little endian

                if (sum(my_crc == crc) == 2)
                    ReceiveData.type = B(1);   
                    if (ReceiveData.type == 40)   %  ACK 
                        FSM_state = 2;
                        if traj_idx < size(new_x_list,2)
                            traj_idx = traj_idx + 1;
                            if (send_left_or_right == 1)
                                X = sprintf('got ACK from MCU, ready to upload left trajectory point %d', traj_idx);
                                disp(X);
                            else
                                X = sprintf('got ACK from MCU, ready to upload right trajectory point %d', traj_idx);
                                disp(X);
                            end
                            continue;
                        else % traj_idx = step+1
                            if (send_left_or_right == 1)
                                send_left_or_right = 2;
                                FSM_state = 2;
                                ack_wait_times = 0;
                                traj_idx = 1;
                                continue;
                            elseif send_left_or_right == 2
                                FSM_state = 4;
                                ack_wait_times = 0;
                                disp('got ACK from MCU, notify finish sending trajectory')
                                continue;
                            end
                        end
                    elseif (ReceiveData.type == 100)
                        X = sprintf('got ERROR from MCU, resend trajectory point %d', traj_idx);
                        disp(X);
                        FSM_state = 2;
                    end
                end
            end
            ack_wait_times = ack_wait_times + 1;
            if (ack_wait_times > MAX_ACK_WAIT_TIMES)
                disp('no ACK!!!! resend current data point')
            	FSM_state = 2;
            end
            
        case 4
            SendData.flag = uint8(170);
            SendData.type = uint8(5);    
            SendData.value = int16(traj_idx); 
            SendData.position = int32(0);
            SendData.velocity = int32(0);
            SendData.crc = pack_and_cal_crc(SendData);
            send_struct(s, SendData);
            disp('notify MCU finish sending trajectory')
            break;
            
    end
    
    
    pause(0.05);
end
fclose(s)
if (ishandle(H) == 0)
    delete(gcf)
end
%% step 2 open port again
% following code need to be manually used 
% fopen(s)

%% step 3 open right grapper
%{
SendData.flag = uint8(170);
SendData.type = uint8(9);    
SendData.value = int16(0); 
SendData.position = int32(0);
SendData.velocity = int32(0);
SendData.crc = pack_and_cal_crc(SendData);
send_struct(s, SendData);
fread(s,14,'uchar')
%}

%% step 4 change state to init state
%{
SendData.flag = uint8(170);
SendData.type = uint8(7);    
SendData.value = int16(traj_idx); 
SendData.position = int32(0);
SendData.velocity = int32(0);
SendData.crc = pack_and_cal_crc(SendData);
send_struct(s, SendData);
fread(s,14,'uchar')
%}

%% step 5 close right grapper
%{
SendData.flag = uint8(170);
SendData.type = uint8(9);    
SendData.value = int16(1); 
SendData.position = int32(0);
SendData.velocity = int32(0);
SendData.crc = pack_and_cal_crc(SendData);
send_struct(s, SendData);
fread(s,14,'uchar')
%}

%% step 6 fire trajectory!
%{
SendData.flag = uint8(170);
SendData.type = uint8(6);    
SendData.value = int16(traj_idx); 
SendData.position = int32(0);
SendData.velocity = int32(0);
SendData.crc = pack_and_cal_crc(SendData);
send_struct(s, SendData);
while (1)
    A = fread(s,1,'uchar');
    if (A(1) == uint8(170)) % the head of a frame 
        B = fread(s,13,'uchar');
        crc = B(12:13);
        crc16 = crcccitt([A(1);B(1:11)]);
        my_crc = [uint8(bi2de(bitget(crc16, 1:8)));uint8(bitshift(crc16, -8))];        % pay attention to the order because of little endian

        if (sum(my_crc == crc) == 2)
            ReceiveData.type = B(1);   
            if (ReceiveData.type == 102)   %  feedback
            end
        end
    end
    pause(0.001);
end
%
fread(s,14,'uchar')
%}
% fclose(s)