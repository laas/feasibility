#!/bin/bash
Nsamples=3000;
rmax=0.5;
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
#for f in ~/git/fastReplanningData/data/fullBodyApprox/fullbody_30_-21_14*;
for f in ~/git/fastReplanningData/data/fullBodyApprox/fullbody*;
do
	BASE=`basename $f`;
	FN="fullBodyApprox/$BASE";
	./bin/13HumanoidsSampler $FN $Nsamples $rmax;

	#octave -q sampling_data_to_neural_network.m
	#"../data/samplingCylinder/h_1_m_0/sampler#"
	#if [[ $BASE =~ (\-?[0-9][0-9]_-?[0-9][0-9]_-?[0-9][0-9]\) ]] ; then echo "jkl:$1"; fi
	if [[ $BASE =~ (\-?[0-9]{1,2}_\-?[0-9]{1,2}_\-?[0-9]{1,2}) ]] ; 
	then 
		i=1;
		NUM=${BASH_REMATCH[$i]};
		SAMPLEFILE="data/13humanoids/sample_$NUM.tmp"; 
		NNfileBASE="feasibility_$NUM";
		NNfile="extern/fann/datasets/$NNfileBASE";
	fi

	##convert sampling file to NN readable file
	rmax=$rmax;
	octave -q scripts/sampling_data_to_neural_network.m "$SAMPLEFILE" "$NNfile" $rmax
	./extern/fann/examples/feasibility $NNfileBASE

done;
