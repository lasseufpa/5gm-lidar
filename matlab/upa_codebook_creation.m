%This script generates UPA codebooks. Will avoid to use structures to
%facilitate importing into Python.
%Will store 2 matrices Wx and Wy with dimensions
%numVectors x Nax a and numVectors x Nay, respectively
%Each codebook entry stores only the indices for picking two columns of
%these matrices and has dimension numVectors x 2
clear *
close all

outputFileNamePrefix = 'upa_codebook_'; %16x16_N'; %.mat';

d=1;
lambdac=2;
normDistanceX=d/lambdac;
normDistanceY=d/lambdac;

Nax=4; % Number of antennas in x axis
Nay=4; % Number of antennas in y

numRandomVectorsX = Nax/2;
numRandomVectorsY = Nay/2;

numDFTPairVectorsX = Nax/2;
numDFTPairVectorsY = Nay/2;

numSteeredVectorsX = Nax;
numSteeredVectorsY = Nay;

numVectorsX = Nax + numRandomVectorsX + numDFTPairVectorsX + numSteeredVectorsX;
numVectorsY = Nay + numRandomVectorsY + numDFTPairVectorsY + numSteeredVectorsY;

Wx = zeros(Nax,numVectorsX);
Wy = zeros(Nay,numVectorsY);

%% DFT vectors
dftMatrixX = ak_fftmtx(Nax,3);
dftMatrixY = ak_fftmtx(Nay,3);

Wx(:,1:Nax)=dftMatrixX; %first entries
Wy(:,1:Nay)=dftMatrixY; %first entries

%% random vectors
%generate single entry (vector) w from a Grassmannian codebook
randomX = randn(Nax,numRandomVectorsX) + 1j*randn(Nax,numRandomVectorsX);
randomY = randn(Nay,numRandomVectorsY) + 1j*randn(Nay,numRandomVectorsY);

Wx(:,Nax+1:Nax+numRandomVectorsX) = randomX;
Wy(:,Nay+1:Nay+numRandomVectorsY) = randomY;

%% pairs of DFT codebooks
dftPairsX = zeros(Nax,numDFTPairVectorsX);
dftPairsY = zeros(Nay,numDFTPairVectorsY);
for i=1:numDFTPairVectorsX
    rand1 = randi(Nax);
    rand2 = randi(Nax);
    dftPairsX(:,i) = dftMatrixX(:,rand1) + dftMatrixX(:,rand2);
end
for i=1:numDFTPairVectorsY
    rand1 = randi(Nay);
    rand2 = randi(Nay);
    dftPairsY(:,i) = dftMatrixY(:,rand1) + dftMatrixY(:,rand2);
end
Wx(:,Nax+numRandomVectorsX+1:Nax+numRandomVectorsX+numDFTPairVectorsX) = dftPairsX;
Wy(:,Nay+numRandomVectorsY+1:Nay+numRandomVectorsY+numDFTPairVectorsY) = dftPairsY;

%% Steered
%simply steer along the y direction with elevation of 90 degrees
minAz=70;
maxAz=110;
pointElevation=pi/2;
steeredX = zeros(Nax,numSteeredVectorsX);
azimuths = deg2rad(linspace(minAz,maxAz,numSteeredVectorsX));
for i=1:numSteeredVectorsX
    pointAzimuth=azimuths(i);
    betax = -2*pi*normDistanceX*sin(pointElevation)*cos(pointAzimuth);
    %linear phase
    wx = exp(1j*(0:Nax-1)*betax);
    steeredX(:,i)=wx;
    %steeredX(:,Nax+numRandomVectorsX+numDFTPairVectorsX+1:end)=wx;
end   
steeredY = zeros(Nay,numSteeredVectorsY);
azimuths = deg2rad(linspace(minAz,maxAz,numSteeredVectorsY));
for i=1:numSteeredVectorsY
    pointAzimuth=azimuths(i);
    betay= -2*pi*normDistanceY*sin(pointElevation)*sin(pointAzimuth);
    %linear phase
    wy = exp(1j*(0:Nay-1)*betay);
    steeredY(:,i)=wy;
    %steeredY(:,Nay+numRandomVectorsY+numDFTPairVectorsY+1:end)=wy;
end
Wx(:,Nax+numRandomVectorsX+numDFTPairVectorsX+1:Nax+numRandomVectorsX+numDFTPairVectorsX+numSteeredVectorsX) = steeredX;
Wy(:,Nay+numRandomVectorsY+numDFTPairVectorsY+1:Nay+numRandomVectorsY+numDFTPairVectorsY+numSteeredVectorsY) = steeredY;

%vectors are column vectors, normalize to unitary norm
Wx = Wx ./ sqrt(sum(abs(Wx).^2));
Wy = Wy ./ sqrt(sum(abs(Wy).^2));

%% Create codebook
%do not combine DFT with random vectors
numCodebookIndices = Nax*Nay + numRandomVectorsX*numRandomVectorsY + Nax*numDFTPairVectorsY + Nay*numDFTPairVectorsX + numSteeredVectorsX*numSteeredVectorsY;

codebook = zeros(numCodebookIndices,2);
numElement=1;
%all DFT combinations
for i=1:Nax
    for j=1:Nay
        codebook(numElement,1) = i;
        codebook(numElement,2) = j;
        numElement = numElement+1;
    end
end
%all random combinations
for i=1:numRandomVectorsX
    for j=1:numRandomVectorsY
        codebook(numElement,1) = Nax+i;
        codebook(numElement,2) = Nay+j;
        numElement = numElement+1;
    end
end
%combine single DFT with one double DFT for x and then for y
for i=1:Nax
    for j=1:numDFTPairVectorsY
        codebook(numElement,1) = i;
        codebook(numElement,2) = Nay+numRandomVectorsY+j;
        numElement = numElement+1;
    end
end
for i=1:Nay
    for j=1:numDFTPairVectorsX
        codebook(numElement,1) = i;
        codebook(numElement,2) = Nax+numRandomVectorsX+j;
        numElement = numElement+1;
    end
end
%steering vectors
for i=1:numSteeredVectorsX
    for j=1:numSteeredVectorsY
        codebook(numElement,1) = Nax+numRandomVectorsX+numDFTPairVectorsX+i;
        codebook(numElement,2) = Nay+numRandomVectorsY+numDFTPairVectorsY+j;
        numElement = numElement+1;
    end
end


%Use the Kronecker to represent the matrix for a pair of wx and wy as a single array
%See  John Brady, Akbar Sayeed, Millimeter-Wave MIMO Transceivers - Chap 10
%Section 10.5
%http://dune.ece.wisc.edu/wp-uploads/2015/11/main_sayeed_brady.pdf
numCodewords = size(codebook,1);
W = zeros(Nax*Nay, numCodewords);
for i=1:numCodewords
    %note that Wy is the first argument to match the Python code
    W(:,i) = kron(Wy(:,codebook(i,2)), Wx(:,codebook(i,1)));
%     if i == 16
%         codebook(i,:)
%     else
%         W(:,i) = zeros(size(W(:,i)));
%     end
end

%save redundant information on Wx and Wy in case they help later
outputFileName = [outputFileNamePrefix num2str(Nax) 'x' num2str(Nay) '_N' num2str(numCodewords) '.mat'];
save(outputFileName,'W','codebook','Wx','Wy','Nax','Nay','-v6')
disp(['Wrote ' outputFileName])

if 0
    W=[];
    codebook=[];
    save('one_antenna_only.mat','W','codebook','Wx','Wy','Nax','Nay','-v6')
end