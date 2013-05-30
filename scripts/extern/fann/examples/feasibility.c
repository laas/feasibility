#include <stdio.h>
#include <string.h>
#include "fann.h"


struct fann_train_data *validate_data;
double low_test_err;
uint test_error_upward;


int FANN_API stopper_criterion(struct fann *ann, struct fann_train_data *train,
	unsigned int max_epochs, unsigned int epochs_between_reports, 
	float desired_error, unsigned int epochs)
{
	double train_err = fann_get_MSE(ann);
	fann_reset_MSE(ann);
	double val_err = fann_test_data(ann, validate_data);
	printf("Epochs     %8d. MSE(train): %.5f. MSE(test): %.5f. Desired-MSE: %.5f\n", epochs, train_err, val_err, desired_error);

	if(val_err <= low_test_err){
		test_error_upward=0;
		low_test_err = val_err;
		return 0;
	}else{
		test_error_upward++;
		if(test_error_upward>5){
			//error on test set goes up -> stop
			return -1;
		}
	}
	return 0;
}
int main(int argc, char** argv)
{
	low_test_err=100000;
	test_error_upward=0;
	printf("%d",argc);
	if(argc!=2){
		printf("usage: feasibility <TrainingDataFile>");
	}
	const char *absoluteFannPath = "/home/aorthey/git/feasibility/scripts/extern/fann/";
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 6;
	const float desired_error = (const float) 0.001;
	const unsigned int max_epochs = 20000;
	const unsigned int epochs_between_reports = 100;
	struct fann *ann;
	struct fann_train_data *train_data, *test_data;
	unsigned int i = 0;

	char trainF[280]={ '\0'};
	char testF[280]={ '\0'};
	char validateF[280]={ '\0'};
	strcat(trainF,absoluteFannPath);
	strcat(testF,absoluteFannPath);
	strcat(validateF,absoluteFannPath);
	strcat(trainF,"datasets/");
	strcat(testF,"datasets/");
	strcat(validateF,"datasets/");
	strcat(trainF,argv[1]);
	strcat(testF,argv[1]);
	strcat(validateF,argv[1]);
	strcat(trainF,".train");
	strcat(testF,".test");
	strcat(validateF,".validate");

	validate_data = fann_read_train_from_file(validateF);
	train_data = fann_read_train_from_file(trainF);
	test_data = fann_read_train_from_file(testF);

	ann = fann_create_standard(num_layers, train_data->num_input, 
				num_neurons_hidden, train_data->num_output);
	fann_set_callback(ann, stopper_criterion);
	fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_learning_momentum(ann, 0.3f);
	//fann_set_training_algorithm(ann, FANN_TRAIN_INCREMENTAL);
	//fann_set_training_algorithm(ann, FANN_TRAIN_RPROP);

	fann_train_on_data(ann, train_data, max_epochs, epochs_between_reports, desired_error);
	printf("MSE error on training data: %f\n", fann_get_MSE(ann));


	printf("Testing network.\n");
	///*

	fann_reset_MSE(ann);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

	struct fann *sparsified = ann;//fann_create_standard(num_layers, train_data->num_input, 
				//num_neurons_hidden, train_data->num_output);

	double error=0;
	uint correct=0;
	uint all = fann_length_train_data(test_data);

	char evaluationMatrix[280]={ '\0'}; //for estimating the decision boundary
	strcat(evaluationMatrix,absoluteFannPath);
	strcat(evaluationMatrix,"datasets/humanoids/");
	strcat(evaluationMatrix,argv[1]);
	strcat(evaluationMatrix,".mat");


	FILE *fid = fopen(evaluationMatrix,"w");
	for(i=0;i<all;i++)
	{
		fann_type *in = test_data->input[i];
		fann_type *t = test_data->output[i];
		fann_run(sparsified, test_data->input[i]);
		fann_type *res = sparsified->output;
		fprintf(fid, "%f %f %f %f %f\n", in[0], in[1], in[2], in[3], res[0]);

		if( res[0]>0 && t[0]>0) correct++;
		else if( res[0]<0 && t[0]<0) correct++;
	}
	fclose(fid);
	printf("input %d, output %d\n", test_data->num_input, test_data->num_output);
	printf("MSE error on test data: %f\n", fann_get_MSE(sparsified));
	printf("error on test data (counting): %f (%d,%d),false %d\n", 1.0-(double)correct/(double)all, correct, all, all-correct);
	printf("Saving network.\n");


	char net[280]={ '\0'}; //for estimating the decision boundary
	strcat(net,absoluteFannPath);
	strcat(net,"datasets/humanoids/");
	strcat(net,argv[1]);
	strcat(net,".net");
	fann_save(sparsified, net);

	fann_destroy_train(test_data);
	printf("Cleaning up.\n");
	//*/
	fann_destroy_train(train_data);
	fann_destroy_train(validate_data);
	fann_destroy(ann);


	return 0;
}
