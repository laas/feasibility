#include <stdio.h>
#include <string.h>
#include "fann.h"

int main(int argc, char** argv)
{
	printf("%d",argc);
	if(argc!=2){
		printf("usage: feasibility <TrainingDataFile>");
	}
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 20;
	const float desired_error = (const float) 0.005;
	const unsigned int max_epochs = 10000;
	const unsigned int epochs_between_reports = 100;
	struct fann *ann;
	struct fann_train_data *train_data, *test_data;
	unsigned int i = 0;

	char trainF[280]={ '\0'};
	char testF[280]={ '\0'};
	strcat(trainF,"/home/aorthey/git/feasibility/scripts/extern/fann/datasets/");
	strcat(testF,"/home/aorthey/git/feasibility/scripts/extern/fann/datasets/");
	strcat(trainF,argv[1]);
	strcat(testF,argv[1]);
	strcat(trainF,".train");
	strcat(testF,".test");

	train_data = fann_read_train_from_file(trainF);
	ann = fann_create_standard(num_layers, train_data->num_input, 
				num_neurons_hidden, train_data->num_output);
	//fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_learning_momentum(ann, 0.6f);

	fann_train_on_data(ann, train_data, max_epochs, epochs_between_reports, desired_error);

	printf("Testing network.\n");
	///*
	test_data = fann_read_train_from_file(testF);

	fann_reset_MSE(ann);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

	struct fann *sparsified = ann;//fann_create_standard(num_layers, train_data->num_input, 
				//num_neurons_hidden, train_data->num_output);

	double error=0;
	uint correct=0;
	uint all = fann_length_train_data(test_data);

	char evaluationMatrix[280]={ '\0'}; //for estimating the decision boundary
	strcat(evaluationMatrix,argv[1]);
	strcat(evaluationMatrix,".mat");
	FILE *fid = fopen(evaluationMatrix,"w");
	for(i=0;i<all;i++)
	{
		fann_type *in = test_data->input[i];
		fann_type *t = test_data->output[i];
		fann_run(sparsified, test_data->input[i]);
		fann_type *res = sparsified->output;
		fprintf(fid, "%f %f %f %f\n", in[0], in[1], in[2], res[0]);

		if( res[0]>0 && t[0]>0) correct++;
		else if( res[0]<0 && t[0]<0) correct++;
	}
	fclose(fid);
	printf("input %d, output %d\n", test_data->num_input, test_data->num_output);
	printf("MSE error on test data: %f\n", fann_get_MSE(sparsified));
	printf("error on test data (counting): %f (%d,%d),false %d\n", 1.0-(double)correct/(double)all, correct, all, all-correct);
	printf("Saving network.\n");

	fann_save(sparsified, "feasibility.net");

	fann_destroy_train(test_data);
	printf("Cleaning up.\n");
	//*/
	fann_destroy_train(train_data);
	fann_destroy(ann);


	return 0;
}
