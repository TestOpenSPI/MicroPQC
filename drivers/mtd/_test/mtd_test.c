// clang-format off
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#if defined (__CC_ARM)
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#elif defined(__GNUC__)
#include <malloc.h>
#endif
#include "board.h"
#include "mtd.h"
#include "hexdump.h"
#include "shell.h"

int example_mtd_help()
{
	printf("Mtd Usage : mtd [option] \n\r");
    printf("Options : \n\r");
    printf("  test                      basic test\n\r");

    printf("  init                      init\n\r");
	printf("  info                      info\n\r");
	printf("  read [address] [length]   read\n\r");
	printf("  write [address] [string]  write\n\r");
	printf("  erase [address] [length]  erase\n\r");

	printf("  writev2 [address] [50]  write 50 byte\n\r"); /* 50 byte 쓰기, write version 2  */
	printf("  writev2 [address] [150]  write 150 byte\n\r"); /* 150 byte 쓰기 */
	printf("  writev2 [address] [256]  write 256 byte\n\r"); /* 256 byte 쓰기 */
	
	return 0;
}

int example_mtd_init()
{
	int ret;
	
	mtd_t *mtd = board_get_mtd_device(MTD_DEVICE_APP);

	ret = mtd_init(mtd);

	if(ret != 0){
		printf("failed(%d)\n\r", ret);
		return 1;
	}

	printf("success\n\r");
	return 0;
}

int example_mtd_read(uint32_t addr, uint32_t len)
{
	uint8_t *buf = malloc(len);
	int ret;
	uint32_t retlen;

	mtd_t *mtd = board_get_mtd_device(MTD_DEVICE_APP);

	if(buf == NULL){
		printf("memory alloc failed \n\r");
		return 1;
	}

	memset(buf, 0x00, len);
	ret = mtd_read(mtd, addr, len, buf, &retlen);
	if (ret != 0)
	{
		printf("mtd_read failed(%d)\n", ret);
	}
	else
	{
		printf("mtd_read: from 0x%08" PRIx32 "(0x%" PRIx32 ")\n", addr, len);
		hexdump(buf, retlen, 0);
	}
	free(buf);

	return 0;
}

int example_mtd_write(uint32_t addr, uint8_t *buf, uint32_t len)
{
	int ret = 1;
	uint32_t retlen;

	mtd_t *mtd = board_get_mtd_device(MTD_DEVICE_APP);

	ret = mtd_write(mtd, addr, len, (const uint8_t *)buf, &retlen);
	if(ret != 0){
		printf("failed(%d)\n\r", ret);
		return 1;
	}

	printf("addr 0x%08" PRIx32 ", buf %p, len %" PRId32 "\n", addr, buf, len);
	printf("success\n\r");

	return 0;
}

int example_mtd_writev2(uint32_t addr, int length) /* sungmin 2023-12-01 write test code */
{
	mtd_t *mtd=board_get_mtd_device(MTD_DEVICE_APP);
	int ret=1;
	uint32_t retlen;
	char test_buf[length];
	
	for(int i=0;i<length;i++)
	{
		test_buf[i]=i%10;
	}

	ret=mtd_write(mtd,addr,length,(const uint8_t *)test_buf,&retlen);
	if(ret!=0)
	{
		printf("failed(%d)\n\r",ret);
		return 1;
	}
	
	return 0;
}

int example_mtd_erase(uint32_t addr, uint32_t len)
{
	int ret = 1;

	mtd_t *mtd = board_get_mtd_device(MTD_DEVICE_APP);

	ret = mtd_erase(mtd, addr, len);
	if(ret != 0){
		printf("failed(%d)\n\r", ret);
		return 1;
	}
	printf("addr 0x%08" PRIx32 ", len %" PRId32 "\n", addr, len);
	printf("success\n\r");

	return 0;
}

int example_mtd_info()
{
	int ret = 1;
	char info[128];

	memset(info, 0, sizeof(info));

	mtd_t *mtd = board_get_mtd_device(MTD_DEVICE_APP);

	ret = mtd_get_info(mtd, info);

	if(ret != 0){
		printf("mtd_get_info failed(%d)\n\r", ret);
		return 1;
	}
		
	printf("mtd_get_info : %s", info); 
	printf("success\n\r");
	
	return 0;
}


int example_mtd_test(int is_all)
{
	int ret = 1;
	uint8_t val = 0;
	uint32_t retlen;
	char info[128];

	mtd_t *mtd = board_get_mtd_device(MTD_DEVICE_APP);

	ret = mtd_init(mtd);
	if(ret != 0){
		printf("mtd_init failed(%d)\n\r", ret);
		return 1;
	}

	memset(info, 0, sizeof(info));

	ret = mtd_get_info(mtd, info);
	if(ret != 0){
		printf("mtd_get_info failed(%d)\n\r", ret);
		return 1;
	}
		
	printf("mtd_get_info : %s", info); 
	printf("success\n\r");

	uint32_t total_size;
	uint32_t erasesize = mtd->erasesize;
	uint8_t *buf = NULL;

	if(is_all){
		total_size = mtd->size;
	}
	else{
		total_size = 1024*1024;
	}

	buf = malloc(erasesize);

	printf("test : total size(0x%"PRIx32"), erasesize(%"PRId32")\n\r", total_size, erasesize);

	// erase
	printf("mtd erase\n\r");
	ret = mtd_erase(mtd, 0, total_size);
	if(ret != 0){
		printf("failed\n\r");
		goto end;
	}

	// read and check
	printf("read and check\n\r");
	for(uint32_t i = 0 ; i < total_size ;){
		memset(buf, 0x00, erasesize);
		mtd_read(mtd, i, erasesize, buf, &retlen);

		for(uint32_t j = 0 ; j < erasesize ; j++){
			if(buf[j] != 0xFF){
				printf("erase compare failed\n\r");
				ret = 1;
				goto end;
			}
		}
		i += erasesize;
	}

	// write
	printf("mtd write\n\r");
	for(uint32_t i = 0 ; i < total_size ; ){
		memset(buf, val, erasesize);
		mtd_write(mtd, i, erasesize, buf, &retlen);
		val++;
		i += erasesize;
	}

	// read and check
	printf("read and check\n\r");
	val = 0;
	for(uint32_t i = 0 ; i < total_size ;){
		memset(buf, 0x00, erasesize);
		mtd_read(mtd, i, erasesize, buf, &retlen);

		for(uint32_t j = 0 ; j < erasesize ; j++){
			if(buf[j] != val){
				printf("erase compare failed\n\r");
				ret = 1;
				goto end;
			}
		}
		i += erasesize;
		val++;
	}

	printf("success\n\r");

	ret = 0;
end:
	if(buf != NULL)
		free(buf);
	
	return ret;
}


int example_mtd(int argc, char *argv[])
{
	(void)argc;
    (void)argv;

    if(argc > 1){
		if(strcmp(argv[1], "test") == 0){
			if(argc == 3 && strcmp(argv[2], "all") == 0)
				example_mtd_test(1);
			else
				example_mtd_test(0);
			
		}
		else if(strcmp(argv[1], "init") == 0){
			example_mtd_init();
		}
		else if(strcmp(argv[1], "info") == 0){
			example_mtd_info();
		}
        else if(strcmp(argv[1], "read") == 0 && argc == 4){
			example_mtd_read(strtol(argv[2], NULL, 16), atoi(argv[3]));
		}
        else if(strcmp(argv[1], "write") == 0 && argc == 4){
			example_mtd_write(strtol(argv[2], NULL, 16), (uint8_t*)argv[3], strlen(argv[3]));
		}
		else if(strcmp(argv[1], "erase") == 0 && argc == 4){
			example_mtd_erase(strtol(argv[2], NULL, 16), atoi(argv[3]));
		}
#if 1 /* bccho, 2023-10-27, 8M 모두 지우기 */		
		else if(!strcmp(argv[1], "erase") && argc == 3 && !strcmp(argv[2], "all")){
			example_mtd_erase(0, 0x800000);
		}
#endif	

#if 1 /* sungmin 2023-12-01, 50, 150, 256 byte mtd write test */
		else if(strcmp(argv[1],"writev2") == 0 && argc ==4){
			example_mtd_writev2(strtol(argv[2],NULL,16),atoi(argv[3]));
		}
#endif
		else{
			example_mtd_help();
		}
	}
	else{
		example_mtd_help();
	}

	return 0;
}

DEFINE_SHELL_CMD(mtd, "mtd commands", example_mtd);

