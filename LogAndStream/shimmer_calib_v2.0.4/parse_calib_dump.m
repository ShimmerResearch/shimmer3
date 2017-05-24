clear all;clc; 

% loadlibrary('sc','shimmer_calibration.h')

fprintf('>>parse_calib_dump.m\n');
format('long');
file_name = 'calib_36ad';

dump_file = fopen(file_name);
total_length = fread(dump_file, 1, 'uint16');
ver.hw_id  = fread(dump_file, 1, 'uint16');
ver.fw_id  = fread(dump_file, 1, 'uint16');
ver.fw_maj = fread(dump_file, 1, 'uint16');
ver.fw_min = fread(dump_file, 1, 'uint8');
ver.fw_int = fread(dump_file, 1, 'uint8');

i = 0;
temp_id = fread(dump_file, 1, 'uint16');
while (temp_id)    
    i = i+1;
    sensor(i).id    = temp_id;
    sensor(i).range = fread(dump_file, 1, 'uint8');
    sensor(i).data_len = fread(dump_file, 1, 'uint8');
    sensor(i).ts    = fread(dump_file, 1, 'uint64');
%     sensor(i).size  = sc_find_size(sensor(i));
    sensor(i).data = fread(dump_file, sensor(i).data_len, 'uint8');
    
    temp_id = fread(dump_file, 1, 'uint16');
end
for i = 1:size(sensor,2)
    sensor(i)
end