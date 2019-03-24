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


%% prepare serial port
s = serial('COM5');
set(s,'BaudRate',115200);
s.ByteOrder = 'littleEndian';
fopen(s);

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

% receive and response 
send_left_or_right = 1;     % if 1, send left trajectory, if 2, send right trajectory
traj_idx = 1;
FSM_state = 1;  % this sender is a finite state machine. 
                % state = 1: wait for the ack of the notification package 
                % state = 2: send a trajectory package
                % state = 3: wait for the ack of the previous trajectory package
ack_wait_times = 0;
MAX_ACK_WAIT_TIMES = 10;   % if ack_wait_times
ReceiveData = struct;                
while (ishandle(H))

    switch FSM_state
        case 1
            % wait for the ack of the notification package 
            A = fread(s,1,'uchar');
            if (A(1) == uint8(170)) % the head of a frame 
                B = fread(s,13,'uchar');
                crc = B(12:13);
                crc16 = crcccitt([A(1);B(1:11)]);
                my_crc = [uint8(bi2de(bitget(crc16, 1:8)));uint8(bitshift(crc16, -8))];        % pay attention to the order because of little endian

                if (sum(my_crc == crc) == 2)
                    ReceiveData.type = B(1);  
                    if (ReceiveData.type == 6)  %  ACK 
                        
                        disp('got ACK from MCU, ready to upload trajectory point 1')
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
            SendData.flag = uint8(170);
            if (send_left_or_right == 1)
                SendData.type = uint8(3);    % type = 3 means this message contains a left trajectory point
                SendData.value = int16(traj_idx);
                SendData.position = int32(new_x_list(2,traj_idx)*1000);
                SendData.velocity = int32(new_x_list(5,traj_idx)*1000);
            else
                SendData.type = uint8(4);    % type = 4 means this message contains a right trajectory point
                SendData.value = int16(traj_idx);
                SendData.position = int32(new_x_list(3,traj_idx)*1000);
                SendData.velocity = int32(new_x_list(6,traj_idx)*1000);
            end
            SendData.crc = pack_and_cal_crc(SendData);
            send_struct(s, SendData);
            
            FSM_state = 3;
            ack_wait_times = 0;
        case 3
            % wait for the ack of the notification package 
            A = fread(s,1,'uchar');
            if (A(1) == uint8(170)) % the head of a frame 
                B = fread(s,13,'uchar');
                crc = B(12:13);
                crc16 = crcccitt([A(1);B(1:11)]);
                my_crc = [uint8(bi2de(bitget(crc16, 1:8)));uint8(bitshift(crc16, -8))];        % pay attention to the order because of little endian

                if (sum(my_crc == crc) == 2)
                    ReceiveData.type = B(1);   
                    if (ReceiveData.type == 6)   %  ACK 
                        FSM_state = 2;
                        traj_idx = traj_idx + 1;
                        disp('got ACK from MCU, ready to upload trajectory point %d', traj_idx)
                        continue;
                    end
                end
            end
            ack_wait_times = ack_wait_times + 1;
            if (ack_wait_times > MAX_ACK_WAIT_TIMES)
                disp('no ACK!!!! resend current data point')
            	FSM_state = 2;
            end
    end
    
    
    pause(0.1);
end
