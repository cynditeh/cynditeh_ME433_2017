function data = read_plot_data(mySerial)
  %nsamples = fscanf(mySerial,'%d');       % first get the number of samples being sent
  nsamples = 100;
  data = zeros(nsamples,3);               % two values per sample:  maf, iir and fir
  times = zeros(nsamples,1);
  for i=1:nsamples
    index(i,:) = fscanf(mySerial,'%d'); % read index
    data(i,:) = fscanf(mySerial,'%d %d %d'); % read in data from PIC32; assume ints, in mA
    times(i) = (i-1)*0.01;                 % 0.01 s between samples
  end
  if nsamples > 1						        
    stairs(times,data(:,1:3));            % plot the reference and actual
  else
    fprintf('Only 1 sample received\n')
    disp(data);
  end
  % print data
  title(sprintf('Filtered result'));
  ylabel('Data ');
  xlabel('Time (ms)');  
end
