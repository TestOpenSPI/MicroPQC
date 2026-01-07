#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "axiocrypto_pqc.h"
#include "axiocrypto_pqc_example.h"
#include "FreeRTOS.h"
#include "task.h"

static const char *hex_msg_vec[] = {
    "D81C4D8D734FCBFBEADE3D3F8A039FAA2A2C9957E835AD55B22E75BF57BB556AC8"
};
static const char *hex_pk_vec[] = {
    "096BA86CB658A8F445C9A5E4C28374BEC879C8655F68526923240918074D0147C03162E4A49200648C652803C6FD7509AE9AA799D6310D0BD42724E0635920186207000767CA5A8546B1755308C304B84FC93B069E265985B398D6B834698287FF829AA820F17A7F4226AB21F601EBD7175226BAB256D8888F009032566D6383D68457EA155A94301870D589C678ED304259E9D37B193BC2A7CCBCBEC51D69158C44073AEC9792630253318BC954DBF50D15028290DC2D309C7B7B02A6823744D463DA17749595CB77E6D16D20D1B4C3AAD89D320EBE5A672BB96D6CD5C1EFEC8B811200CBB062E473352540EDDEF8AF9499F8CDD1DC7C6873F0C7A6BCB7097560271F946849B7F373640BB69CA9B518AA380A6EB0A7275EE84E9C221AED88F5BFBAF43A3EDE8E6AA42558104FAF800E018441930376C6F6E751569971F47ADBCA5CA00C801988F317A18722A29298925EA154DBC9024E120524A2D41DC0F18FD8D909F6C50977404E201767078BA9A1F9E40A8B2BA9C01B7DA3A0B73A4C2A6B4F518BBEE3455D0AF2204DDC031C805C72CCB647940B1E6794D859AAEBCEA0DEB581D61B9248BD9697B5CB974A8176E8F910469CAE0AB4ED92D2AEE9F7EB50296DAF8057476305C1189D1D9840A0944F0447FB81E511420E67891B98FA6C257034D5A063437D379177CE8D3FA6EAF12E2DBB7EB8E498481612B1929617DA5FB45E4CDF893927D8BA842AA861D9C50471C6D0C6DF7E2BB26465A0EB6A3A709DE792AAFAAF922AA95DD5920B72B4B8856C6E632860B10F5CC08450003671AF388961872B466400ADB815BA81EA794945D19A100622A6CA0D41C4EA620C21DC125119E372418F04402D9FA7180F7BC89AFA54F8082244A42F46E5B5ABCE87B50A7D6FEBE8D7BBBAC92657CBDA1DB7C25572A4C1D0BAEA30447A865A2B1036B880037E2F4D26D453E9E913259779E9169B28A62EB809A5C744E04E260E1F2BBDA874F1AC674839DDB47B3148C5946DE0180148B7973D63C58193B17CD05D16E80CD7928C2A338363A23A81C0608C87505589B9DA1C617E7B70786B6754FBB30A5816810B9E126CFCC5AA49326E9D842973874B6359B5DB75610BA68A98C7B5E83F125A82522E13B83FB8F864E2A97B73B5D544A7415B6504A13939EAB1595D64FAF41FAB25A864A574DE524405E878339877886D2FC07FA0311508252413EDFA1158466667AFF78386DAF7CB4C9B850992F96E20525330599AB601D454688E294C8C3E"
};
static const char *hex_sk_vec[] = {
    "59044102F3CFBE1BE03C144102F7EF75FBEF83043F7CFC20C20BEEC007DE3F041FBF0BFF401041030C40040FAE7E103F7E100085FC013D1410C80C2F000810461C2F480BEE8017D17F07F1411BA24013C1BDF83DC407D17E07C13917F0F9044045FC40BD0FF07D07EF0003DFC1F3CFFD1FC03FEFC0B8FC6E7B0BBDBD0FE0BE17D14307EFFE0FBFC6F81FBFF43EC1F87041D42083EC3DC2F4407BF84EC4140FC403F037F3FEC013E0FEE02180082F83FBE07BFFE043F40EC6FFB1BF200007FFBFFA0FFF6FFBCE83EBFEBEFC0FFDF3F103FC6F3FF0500A18718308007D03F200E4213BF04FFD17D000F0017A17F180E04FFF07DEC2244048148E8704503EE06F86080243F81FFF03BF4003F07EF3DE02FBFFC00420C1F40FBDF0707E043FF5FFD0000430400C4F49F4207C142F80EC3E010BFF7C13F07FF85F7F17E07C17FF33FC4EC303FFBCFFEEC41830FF0831BDF45F05F06FC503B0C0F84E4013E100E7E1441450C2FBEEBC0C0FBEFC60BCFFEF3CFBDF4303EF800BF2BE0BF001F01F43F41FFE08517B001141E00144F7EF8007CEBDFFFF4213A0B9F8A0FE04103C17E0820BB1C30C30C00FFFFC00007D18017CFF90C3101E7E103040FC4FBE04213E07AF80FFEFC80FBFBD0810BCFB8FBC087FB8FFF1010C2E81002F3EF3BF01F07E41FBC07F2C0FB8F43F401C5D81FFCEBE07C07E0BF17EEBEE830C514003FF7EF3E08403D1FFFFE105F840C20BDF0607FFFEF46E7EFFF08000400DE830000F3F82EF9D82E84EFFF3CEC4E81E01002103102EFC080F3B0801041BAE42F7F040F83EC31010031BC0410FAFF9F0004010133A089FFEF7BE8317A0020FEF010052BA04107E100F821C2F41F44F4EF7B000F02E41F82F380830FE08A1F707FF82EC7F42E81004041103E8307B13D0FDFF8F830F9FC5FFCD7E040F410FFFB9F423750860C11C5FFA144EC0080F02DC0F420820450790020BCF80EFFFBCEC4FBFF4200AFC00C02060C004303EF81FFA104107E4117AF01F81202FC1E44143FFE206EB3E881BB13F13920403FF7A000144102E7FFC2143E7FF4AF3F13F07E181DC317E240F4500303F2DDDCF1E1513E3EF15E8DC1309E50AEE03EFDC17081706FD03E6ECE4F30EBD1909051906E90CE806EB0B19E719EFFBF10D0DF1DC0CF6F1F4F8FEFBE9F9550E2107FCDCCBDFE9F4F7EE1AF8142115F910002AF2F5FF141ADA220AECFE040CEF0B29EB201930F2D3E401E5DEEFF4DDEA17F1FE141217F81C36050109F8F61F02DD19F90310C7F40208E9052C3942F8FFF2CCF9FDF83CFA12DC091C0D02F00411F5281E40D7F92DBA11D73D04C10BFD13E617110AF3ED05F6CFE705E0F70E1FF80533FC120C002CE81FF52638190FE3FED6F0FBBB23E6F408EF32220B13DD27F007E5FA00D72614F0E302210707EC111E070E2A032DF91DE3FCE800F1F9F2F7FE170101180412CBD1E90019F2011522DAEAED13F8E5F425DCEF24E01CE614E7DCEC01F2F4F914F4010107ED26E2E9DF0BF5F007EA07FAFBC6D7E607FAFCFD270DFD0D17FC4EF0EE00071AECDE09F8F215E113F80209CCF308D7E6251ECE0EDFED0CC9F4050B2714F61BF703F0EBF104010DEBFBF21AFC1BF01823FEDEFAF7F807E3F3020AEB01FE19EEE8E90D00E5FAED1EFDF628E5F0E6F0FC13F4FB05FB0B09EA0A0E08EE13293212E90CE4FEF223F4FF030BEBED1B402ED2F6171102BC0CF9E9F335ED0C01FAF0FEFAE41DF0050A162C11171CD90BEE211218EDFAFA0F03F4171412F319D60B01FAEE1F2823F0D6EF12D6DFEAFBFC170DECDA06E7CED500031E"
};
static const char *hex_sm_vec[] = {
    "026833B3C07507E4201748494D832B6EE2A6C93BFF9B0EE343B550D1F85A3D0DE0D704C6D17842951309D81C4D8D734FCBFBEADE3D3F8A039FAA2A2C9957E835AD55B22E75BF57BB556AC8290765843D1E460D17A527D2BCA405BD55BBC7DA09A8C620BE0AF4A767D9DB96B80F55E466676751EAABA7B93B86D71132DAA0EB376782B9EEE37519CE10FDD33FE9F29312C31D8736206D165CF4C528AA3DDC017845E1F0DD5B0A44FF961C42D874A95533E5B438982F524CA954D87533BFBE42C63FF2ABC77A34C79DB55A99171BBCB72C842A6530AF2F753F0C34AC632F9F1E7949F0BF6C67665B27722A8857D626B6FF1A136D923A39F4069B7477FF946E5247A6627791D49B59EDC9E2525A860E6E9828D18F64A9F17222E8166A02453859BBDA0B8186D8C9928BB571E4146401D7430E225904673AD21CCAC54C146C248A1DD69AB6491E901D6D71B152155BE97DE057F3916A3F1B4273308C29B2F4D9697167B90681B1583ED930A71E990467DEA368134BECEEBD597F9BEC922E816F1B0570D728F4AE0464C1F797657F87A4E52DCDCAEB9272662EA66D7C6CD8781B31AF555AD93F5F65E75816CB8DC306BB67E592B5261BACA7C509629EA2AF8ABB80CBA89EE535B76DFD9CCBBE3BF48F2BC8AA34B26E1103291053F5CB8DE3A45AFA5A76DF8B2122ED2C82FBCF2259290D41A14F86B12F35F5D49762B34CFF13EE7E42EDEC70201D7F37C33316288FA3078E36E58108865C3CFE263D563692043DECC62F3426F86061285B7B1B336F56FF41BB65E9CD6D9B92FD90F864AA1C923CB8C755F5CDE1770D862595427149D7721AAAB5D194AEA9ACDECA15BE43CBA6A62B5A33909E9FC4DA1C5814FBD7CD6A2FA572E318B42C6C319140B86E66392580A11A2B431F44C1F9270E4F7B2490F3B325A9977A71A575915636635B9969DBD6D220B24C3D99CEBBBD834B88222BD08C3ABE124E80"
};

int axiocrypto_pqc_falcon512_kat(void)
{
    int ret = -1;
    size_t i;
    size_t failures = 0;

    const size_t n_pk  = ARRAY_LEN(hex_pk_vec);
    const size_t n_sk  = ARRAY_LEN(hex_sk_vec);
    const size_t n_msg = ARRAY_LEN(hex_msg_vec);
    const size_t n_sm  = ARRAY_LEN(hex_sm_vec);

    uint8_t *pk = NULL, *sk = NULL, *msg = NULL, *sm = NULL;
    uint8_t *out_signed = NULL, *recovered = NULL;

    size_t msg_len = 0;
    size_t sm_len = 0;
    size_t recovered_len = 0;
    size_t out_signed_len = 0;

    const size_t out_signed_buf_len = (size_t)PQC_MSG_MAX_SIZE
                                    + (size_t)PQC_FALCON512_SIGNATURE_SIZE
                                    + 1024;

    printf("================================================================================\n");

    /* 배열 정합성 체크 */
    if (n_pk == 0 || n_pk != n_sk || n_pk != n_msg || n_pk != n_sm) {
        printf("FAIL [INIT] vector count mismatch: pk=%u sk=%u msg=%u sm=%u\n",
               (unsigned)n_pk, (unsigned)n_sk, (unsigned)n_msg, (unsigned)n_sm);
        return -1;
    }

    pk = (uint8_t*)malloc(PQC_FALCON512_PUBLICKEY_SIZE);
    if (!pk) { printf("FAIL [INIT] pk malloc failed\n"); goto cleanup; }

    sk = (uint8_t*)malloc(PQC_FALCON512_SECRETKEY_SIZE);
    if (!sk) { printf("FAIL [INIT] sk malloc failed\n"); goto cleanup; }

    msg = (uint8_t*)malloc(PQC_MSG_MAX_SIZE);
    if (!msg) { printf("FAIL [INIT] msg malloc failed\n"); goto cleanup; }

    sm = (uint8_t*)malloc(out_signed_buf_len);
    if (!sm) { printf("FAIL [INIT] sm malloc failed\n"); goto cleanup; }

    out_signed = (uint8_t*)malloc(out_signed_buf_len);
    if (!out_signed) { printf("FAIL [INIT] out_signed malloc failed\n"); goto cleanup; }

    recovered = (uint8_t*)malloc(PQC_MSG_MAX_SIZE);
    if (!recovered) { printf("FAIL [INIT] recovered malloc failed\n"); goto cleanup; }

    for (i = 0; i < n_pk; i++) {
        /* 출력 버퍼 초기화 */
        memset(pk, 0, PQC_FALCON512_PUBLICKEY_SIZE);
        memset(sk, 0, PQC_FALCON512_SECRETKEY_SIZE);
        memset(msg, 0, PQC_MSG_MAX_SIZE);
        memset(sm, 0, out_signed_buf_len);
        memset(out_signed, 0, out_signed_buf_len);
        memset(recovered, 0, PQC_MSG_MAX_SIZE);

        msg_len = 0;
        sm_len = 0;

        /* 벡터 파싱 */
        if (hexTobin(hex_pk_vec[i], pk, PQC_FALCON512_PUBLICKEY_SIZE) != (int)PQC_FALCON512_PUBLICKEY_SIZE) {
            printf("FAIL [#%02u][INIT] pk hex parse/size mismatch\n", (unsigned)i);
            failures++;
            continue;
        }

        if (hexTobin(hex_sk_vec[i], sk, PQC_FALCON512_SECRETKEY_SIZE) != (int)PQC_FALCON512_SECRETKEY_SIZE) {
            printf("FAIL [#%02u][INIT] sk hex parse/size mismatch\n", (unsigned)i);
            failures++;
            continue;
        }

        msg_len = strlen(hex_msg_vec[i]) / 2;
        if (msg_len > PQC_MSG_MAX_SIZE) {
            printf("FAIL [#%02u][INIT] msg too large: %u bytes (max %u)\n",
                   (unsigned)i, (unsigned)msg_len, (unsigned)PQC_MSG_MAX_SIZE);
            failures++;
            continue;
        }
        if (hexTobin(hex_msg_vec[i], msg, msg_len) != (int)msg_len) {
            printf("FAIL [#%02u][INIT] msg hex parse/size mismatch\n", (unsigned)i);
            failures++;
            continue;
        }

        sm_len = strlen(hex_sm_vec[i]) / 2;
        if (sm_len > out_signed_buf_len) {
            printf("FAIL [#%02u][INIT] sm too large: %u bytes (buf %u)\n",
                   (unsigned)i, (unsigned)sm_len, (unsigned)out_signed_buf_len);
            failures++;
            continue;
        }
        if (hexTobin(hex_sm_vec[i], sm, sm_len) != (int)sm_len) {
            printf("FAIL [#%02u][INIT] sm hex parse/size mismatch\n", (unsigned)i);
            failures++;
            continue;
        }

        /* ---- [VEC] open(pk_vec, sm_vec) => recovered == msg_vec ---- */
        printf("[#%02u][VEC] open(provided PK, provided SM) matches provided MSG..............", (unsigned)i);

        recovered_len = PQC_MSG_MAX_SIZE;
        ret = axiocrypto_pqc_open(PQC_FN_DSA_512,
                                  pk, PQC_FALCON512_PUBLICKEY_SIZE,
                                  sm, sm_len,
                                  recovered, &recovered_len);
        if (ret != PQC_SUCCESS) {
            printf("FAIL (open ret=%d)\n", ret);
            failures++;
            continue;
        }
        if (recovered_len != msg_len || memcmp(recovered, msg, msg_len) != 0) {
            printf("FAIL (recovered message mismatch)\n");
            failures++;
            continue;
        }
        printf("PASS\n");

        /* ---- [RT] sign(sk_vec, msg_vec) -> open(pk_vec, out_signed) ---- */
        printf("[#%02u][RT ] sign(provided SK, provided MSG) then open(provided PK) matches...", (unsigned)i);

        out_signed_len = out_signed_buf_len;
        ret = axiocrypto_pqc_sign(PQC_FN_DSA_512,
                                  sk, PQC_FALCON512_SECRETKEY_SIZE,
                                  msg, msg_len,
                                  out_signed, &out_signed_len);
        if (ret != PQC_SUCCESS) {
            printf("FAIL (sign ret=%d)\n", ret);
            failures++;
            continue;
        }

        recovered_len = PQC_MSG_MAX_SIZE;
        ret = axiocrypto_pqc_open(PQC_FN_DSA_512,
                                  pk, PQC_FALCON512_PUBLICKEY_SIZE,
                                  out_signed, out_signed_len,
                                  recovered, &recovered_len);
        if (ret != PQC_SUCCESS) {
            printf("FAIL (open ret=%d)\n", ret);
            failures++;
            continue;
        }
        if (recovered_len != msg_len || memcmp(recovered, msg, msg_len) != 0) {
            printf("FAIL (round-trip recovered message mismatch)\n");
            failures++;
            continue;
        }
        printf("PASS\n");
    }

    if (failures == 0) {
        printf("RESULT: SUCCESS (%u/%u passed)\n", (unsigned)n_pk, (unsigned)n_pk);
        ret = 0;
    } else {
        printf("RESULT: FAILED (%u/%u failed)\n", (unsigned)failures, (unsigned)n_pk);
        ret = -1;
    }
    printf("================================================================================\n\n");

cleanup:
    if (pk) free(pk);
    if (sk) free(sk);
    if (msg) free(msg);
    if (sm) free(sm);
    if (out_signed) free(out_signed);
    if (recovered) free(recovered);

    return ret;
}

void axiocrypto_pqc_falcon512_bench()
{
    int ret = 0;

    uint8_t pk[PQC_FALCON512_PUBLICKEY_SIZE] = {0x00,};
    uint8_t sk[PQC_FALCON512_SECRETKEY_SIZE] = {0x00,};
    uint8_t sig[PQC_MSG_MAX_SIZE + PQC_FALCON512_SIGNATURE_SIZE + 1024] = {0x00,};
    size_t siglen = PQC_MSG_MAX_SIZE + PQC_FALCON512_SIGNATURE_SIZE + 1024;
    uint8_t msg[PQC_MSG_MAX_SIZE] = {0x00,};
    size_t msglen = PQC_MSG_MAX_SIZE;

    uint8_t sm[PQC_MSG_MAX_SIZE + PQC_FALCON512_SIGNATURE_SIZE + 1024] = {0x00,};
    size_t smlen = PQC_MSG_MAX_SIZE + PQC_FALCON512_SIGNATURE_SIZE + 1024;
    

    TickType_t s, e;

    printf("(op=%d, msglen=%d)\n", NTESTS, (int)msglen);
    printf("============================================================\n");
    secure_stack_init();
    s = xTaskGetTickCount();
    for(int i = 0; i < NTESTS; i++){
        ret = axiocrypto_pqc_keypair(PQC_FN_DSA_512, pk, PQC_FALCON512_PUBLICKEY_SIZE, sk, PQC_FALCON512_SECRETKEY_SIZE);
        if(ret != PQC_SUCCESS){
            printf("FALCON512 keypair generation failed(ret:%d)\n", ret);
            return;
        }
    }
    e = xTaskGetTickCount();
    printf("KeyGen:\t");
    print_elapsed_ms_ticks(s, e, NTESTS);
    printf(" | ");
    secure_stack_disp();

    secure_stack_init();
    s = xTaskGetTickCount();
    for(int i = 0; i < NTESTS; i++){
        siglen = PQC_MSG_MAX_SIZE + PQC_FALCON512_SIGNATURE_SIZE + 1024;
        ret = axiocrypto_pqc_sign(PQC_FN_DSA_512, sk, PQC_FALCON512_SECRETKEY_SIZE, msg, msglen, sig, &siglen);
        if(ret != PQC_SUCCESS){
            printf("FALCON512 signing failed(ret:%d)\n", ret);
            return;
        }
    }
    e = xTaskGetTickCount();
    printf("Sign:\t");
    print_elapsed_ms_ticks(s, e, NTESTS);
    printf(" | ");
    secure_stack_disp();

    secure_stack_init();
    s = xTaskGetTickCount();
    for(int i = 0; i < NTESTS; i++){
        memcpy(sm, sig, siglen);
        smlen = siglen;
        msglen = PQC_MSG_MAX_SIZE;
        ret = axiocrypto_pqc_open(PQC_FN_DSA_512, pk, PQC_FALCON512_PUBLICKEY_SIZE, sm, smlen, msg, &msglen);
        if(ret != PQC_SUCCESS){
            printf("FALCON512 signature verification failed(ret:%d)\n", ret);
            return;
        }
    }
    e = xTaskGetTickCount();
    printf("Verify:\t");
    print_elapsed_ms_ticks(s, e, NTESTS);
    printf(" | ");
    secure_stack_disp();
    printf("============================================================\n\n");

    return;
}
