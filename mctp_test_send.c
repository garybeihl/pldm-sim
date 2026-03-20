/* Minimal test to diagnose AF_MCTP sendto on guest */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/mctp.h>

int main(void)
{
	int sd, rc, val;
	struct sockaddr_mctp_ext addr;
	uint8_t data[] = { 0x00, 0x80, 0x02 }; /* GetEID */

	sd = socket(AF_MCTP, SOCK_DGRAM, 0);
	if (sd < 0) {
		perror("socket");
		return 1;
	}
	printf("socket OK: fd=%d\n", sd);

	/* Set extended addressing */
	val = 1;
	rc = setsockopt(sd, SOL_MCTP, MCTP_OPT_ADDR_EXT, &val, sizeof(val));
	if (rc < 0) {
		perror("setsockopt MCTP_OPT_ADDR_EXT");
		close(sd);
		return 1;
	}
	printf("setsockopt MCTP_OPT_ADDR_EXT OK\n");

	/* Send to EID 0 with physical addressing */
	memset(&addr, 0, sizeof(addr));
	addr.smctp_base.smctp_family = AF_MCTP;
	addr.smctp_base.smctp_network = MCTP_NET_ANY;
	addr.smctp_base.smctp_addr.s_addr = 0; /* NULL EID */
	addr.smctp_base.smctp_type = 0; /* MCTP control */
	addr.smctp_base.smctp_tag = MCTP_TAG_OWNER;

	/* Try ifindex 8 (current mctpserial0) */
	addr.smctp_ifindex = 8;
	addr.smctp_halen = 0;

	printf("Sending to EID 0 via ifindex 8...\n");
	rc = sendto(sd, data, sizeof(data), 0,
		    (struct sockaddr *)&addr, sizeof(addr));
	if (rc < 0) {
		printf("sendto failed: %s (errno=%d)\n", strerror(errno), errno);
	} else {
		printf("sendto OK: sent %d bytes\n", rc);
	}

	/* Also try without extended addressing */
	val = 0;
	setsockopt(sd, SOL_MCTP, MCTP_OPT_ADDR_EXT, &val, sizeof(val));

	struct sockaddr_mctp base_addr;
	memset(&base_addr, 0, sizeof(base_addr));
	base_addr.smctp_family = AF_MCTP;
	base_addr.smctp_network = 1;
	base_addr.smctp_addr.s_addr = 0;
	base_addr.smctp_type = 0;
	base_addr.smctp_tag = MCTP_TAG_OWNER;

	printf("Sending to EID 0 without ext addr (net=1)...\n");
	rc = sendto(sd, data, sizeof(data), 0,
		    (struct sockaddr *)&base_addr, sizeof(base_addr));
	if (rc < 0) {
		printf("sendto failed: %s (errno=%d)\n", strerror(errno), errno);
	} else {
		printf("sendto OK: sent %d bytes\n", rc);
	}

	close(sd);
	return 0;
}
