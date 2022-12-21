function data = parsedata(rawdata)
    split_rawdata = str2double(split(rawdata));
    %disp(split_rawdata)
    data = split_rawdata(2:end-1);
end