#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "shell.h"
#include "axiocrypto_pqc.h"
#include "axiocrypto_pqc_example.h"

void axiocrypto_pqc_example_help();

int axiocrypto_pqc_example(int argc, char *argv[])
{
    int help_flag = 0;

    if (argc <= 1) {
        help_flag = 1;
    }
	else if(strcmp(argv[1], "kat") == 0){
		if (argc <= 2) {
            help_flag = 1;
        }
        else if(strcmp(argv[2], "all") == 0){
            axiocrypto_pqc_kat_all();
        }
        else if (strcmp(argv[2], "list") == 0) {
            axiocrypto_pqc_kat_list();
        }
        else if(axiocrypto_pqc_kat_one(argv[2]) != 0){
            help_flag = 1;
        }
	}
	else if(strcmp(argv[1], "bench" ) == 0){
        if (argc <= 2) {
            help_flag = 1;
        }
        else if(strcmp(argv[2], "all") == 0){
            axiocrypto_pqc_bench_all();
        }
        else if (strcmp(argv[2], "list") == 0) {
            axiocrypto_pqc_bench_list();
        }
        else if(axiocrypto_pqc_bench_one(argv[2]) != 0){
            help_flag = 1;
        }
    }
	else{
		help_flag = 1;
	}

    if(help_flag){
        axiocrypto_pqc_example_help();
    }

    return 0;
}

void axiocrypto_pqc_example_help()
{
    printf("PQC Example : pqc [option] \n\r");
    printf("Options : \n\r");
    printf("  kat list              kat alg list\n\r");
    printf("  kat all               kat all alg\n\r");
    printf("  kat <algname>         kat select alg\n\r");
    printf("  bench list            benchmark alg list\n\r");
    printf("  bench all             benchmark all alg\n\r");
    printf("  bench <algname>       benchmark select alg\n\r");
    
    return;
}

DEFINE_SHELL_CMD(pqc, "AxioCrypto PQC Example", axiocrypto_pqc_example);
