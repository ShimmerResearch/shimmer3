function size = sc_find_size(sensor) 

run shimmer_calibration.m;

switch sensor.id
    case SC.SENSOR_ANALOG_ACCEL 
        size = 21;
    case SC.SENSOR_MPU9150_GYRO 
        size = 21;
    case SC.SENSOR_LSM303DLHC_ACCEL 
        size = 21;
    case SC.SENSOR_LSM303DLHC_MAG 
        size = 21;
    case SC.SENSOR_BMP180_PRESSURE 
        size = 22;
    otherwise
        size = 0;
end

end