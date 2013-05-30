#!/bin/bash
Nsamples=5000;
rmax=0.5;
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
for f in ~/git/fastReplanningData/data/fullBodyApprox/fullbody_30_-21_14*;
do
	BASE=`basename $f`;
	FN="fullBodyApprox/$BASE";
	./bin/MCMCSampler $FN 0.01 0.0 $Nsamples $rmax;


	#octave -q sampling_data_to_neural_network.m
	#"../data/samplingCylinder/h_1_m_0/sampler#"
	#if [[ $BASE =~ (\-?[0-9][0-9]_-?[0-9][0-9]_-?[0-9][0-9]\) ]] ; then echo "jkl:$1"; fi
	if [[ $BASE =~ (\-?[0-9][0-9]_\-?[0-9][0-9]_\-?[0-9][0-9]) ]] ; 
	then 
		i=1;
		NUM=${BASH_REMATCH[$i]};
		SAMPLEFILE="data/cylinderSpecial/h_1_m_0/sample_$NUM.tmp"; 
		NNfileBASE="feasibility_$NUM";
		NNfile="scripts/extern/fann/datasets/$NNfileBASE";
	fi

	##convert sampling file to NN readable file
	rmax=$rmax+0.2;
	octave -q scripts/sampling_data_to_neural_network.m "$SAMPLEFILE" "$NNfile" $rmax

	./scripts/extern/fann/examples/feasibility $NNfileBASE
	#sprintf(cmd2, "./scripts/extern/fann/examples/feasibility %s", nn_file_name.c_str());

done;
