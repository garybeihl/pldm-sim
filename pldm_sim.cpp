/*
 * pldm_sim.cpp - PLDM Device Simulator for integration testing
 *
 * This program emulates a PLDM firmware device that communicates with pldmd
 * (the Update Agent) over MCTP serial. Device behavior is driven by JSON
 * scenario files, allowing different test configurations without recompiling.
 *
 * Supported PLDM types: Base, Firmware Update, Platform Monitoring
 *
 * Transport: AF_MCTP socket → mctp-serial kernel module → PTY → QEMU UART
 *
 * MCTP control messages (endpoint discovery) are handled via AF_PACKET to
 * work around host kernels < 6.18 that lack NULL EID routing support.
 * PLDM messages use AF_MCTP sockets normally (they use real EIDs).
 *
 * Usage: sudo ./pldm_sim [--config scenario.json] /dev/pts/N [mctp-interface]
 */

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/mctp.h>
#include <linux/tty.h>

#include <libpldm/firmware_fd.h>
#include <libpldm/firmware_update.h>
#include <libpldm/base.h>
#include <libpldm/platform.h>
#include <libpldm/entity.h>

#include <nlohmann/json.hpp>

using Json = nlohmann::json;

/* BMC (pldmd) EID */
static constexpr uint8_t BMC_EID = 8;

/* MCTP message types */
static constexpr uint8_t MCTP_TYPE_CONTROL = 0x00;
static constexpr uint8_t MCTP_TYPE_PLDM = 0x01;

/* MCTP control commands */
static constexpr uint8_t MCTP_CTRL_GET_EID = 0x02;
static constexpr uint8_t MCTP_CTRL_SET_EID = 0x01;
static constexpr uint8_t MCTP_CTRL_GET_UUID = 0x03;
static constexpr uint8_t MCTP_CTRL_GET_MSG_TYPE = 0x05;

/* MCTP transport header fields */
static constexpr uint8_t MCTP_HDR_VERSION = 0x01;
static constexpr uint8_t MCTP_HDR_FLAG_SOM = 0x80;
static constexpr uint8_t MCTP_HDR_FLAG_EOM = 0x40;
static constexpr uint8_t MCTP_HDR_FLAG_TO = 0x08;
static constexpr uint8_t MCTP_HDR_TAG_MASK = 0x07;

/* ETH_P_MCTP for AF_PACKET */
#ifndef ETH_P_MCTP
#define ETH_P_MCTP 0x00FA
#endif

static volatile sig_atomic_t running = 1;
static const char *mctp_tool_path;

/* ===== Configuration structs ===== */

struct SimComponent {
	uint16_t id = 0;
	uint16_t classification = 10;
	std::string version;
	bool canUpdate = true;
	uint8_t rejectReason = PLDM_CRC_COMP_CAN_BE_UPDATED;
	uint8_t verifyResult = PLDM_FWUP_VERIFY_SUCCESS;
	uint8_t applyResult = PLDM_FWUP_APPLY_SUCCESS;
	bool activateSelf = true;
};

struct SimConfig {
	uint8_t eid = 20;
	uint8_t tid = 1;
	std::array<uint8_t, 16> uuid = {
		0x16, 0x20, 0x23, 0xC9, 0x3E, 0xC5, 0x41, 0x15,
		0x95, 0xF4, 0x48, 0x70, 0x1D, 0x49, 0xD6, 0x75
	};
	std::string name = "MockDevice";
	uint16_t entityType = PLDM_ENTITY_BOARD;
	uint32_t maxTransferSize = 4096;
	std::array<uint8_t, 3> pldmVersion = {0xF1, 0xF1, 0xF0};
	std::string imageSetVersion = "1.0.0";
	std::vector<SimComponent> components;
	bool platformEnabled = false;
	uint32_t idleTimeoutMs = 120000;
	uint32_t retryTimeMs = 3000;
};

/* Application context */
struct SimContext {
	int pldm_sock;
	int ctrl_sock;
	int raw_sock;
	int tty_fd;
	int ifindex;
	struct pldm_fd *fd;
	uint32_t fw_data_received;
	SimConfig cfg;

	/* Dynamic component table built from cfg */
	std::vector<pldm_firmware_component_standalone> compStandalone;
	std::vector<const pldm_firmware_component_standalone *> compPtrs;

	/* PDR repository */
	uint8_t pdrRecords[2][512];
	size_t pdrRecordSizes[2];
	int pdrRecordCount;
};

/* ===== Helper functions ===== */

static bool parseUuid(const std::string &s, std::array<uint8_t, 16> &out)
{
	std::string hex;
	for (char c : s) {
		if (c != '-')
			hex += c;
	}
	if (hex.size() != 32)
		return false;
	for (int i = 0; i < 16; i++) {
		unsigned int byte;
		if (sscanf(hex.c_str() + i * 2, "%2x", &byte) != 1)
			return false;
		out[static_cast<size_t>(i)] = static_cast<uint8_t>(byte);
	}
	return true;
}

static bool parsePldmVersion(const std::string &s,
			     std::array<uint8_t, 3> &out)
{
	unsigned int major, minor, update;
	if (sscanf(s.c_str(), "%u.%u.%u", &major, &minor, &update) != 3)
		return false;
	out[0] = static_cast<uint8_t>(0xF0 | (major & 0x0F));
	out[1] = static_cast<uint8_t>(0xF0 | (minor & 0x0F));
	out[2] = static_cast<uint8_t>(0xF0 | (update & 0x0F));
	return true;
}

static uint8_t parseRejectReason(const std::string &s)
{
	if (s == "COMP_COMPARISON_STAMP_IDENTICAL")
		return PLDM_CRC_COMP_COMPARISON_STAMP_IDENTICAL;
	if (s == "COMP_COMPARISON_STAMP_LOWER")
		return PLDM_CRC_COMP_COMPARISON_STAMP_LOWER;
	if (s == "COMP_CONFLICT")
		return PLDM_CRC_COMP_CONFLICT;
	if (s == "COMP_PREREQUISITES_NOT_MET")
		return PLDM_CRC_COMP_PREREQUISITES_NOT_MET;
	if (s == "COMP_NOT_SUPPORTED")
		return PLDM_CRC_COMP_NOT_SUPPORTED;
	if (s == "COMP_SECURITY_RESTRICTIONS")
		return PLDM_CRC_COMP_SECURITY_RESTRICTIONS;
	if (s == "COMP_VER_STR_IDENTICAL")
		return PLDM_CRC_COMP_VER_STR_IDENTICAL;
	if (s == "COMP_VER_STR_LOWER")
		return PLDM_CRC_COMP_VER_STR_LOWER;
	fprintf(stderr,
		"[pldm-sim] Unknown reject_reason: %s, "
		"using COMP_NOT_SUPPORTED\n",
		s.c_str());
	return PLDM_CRC_COMP_NOT_SUPPORTED;
}

static uint8_t parseVerifyResult(const std::string &s)
{
	if (s == "SUCCESS")
		return PLDM_FWUP_VERIFY_SUCCESS;
	if (s == "VERIFICATION_FAILURE")
		return PLDM_FWUP_VERIFY_ERROR_VERIFICATION_FAILURE;
	if (s == "VERSION_MISMATCH")
		return PLDM_FWUP_VERIFY_ERROR_VERSION_MISMATCH;
	if (s == "FAILED_FD_SECURITY_CHECKS")
		return PLDM_FWUP_VERIFY_FAILED_FD_SECURITY_CHECKS;
	if (s == "IMAGE_INCOMPLETE")
		return PLDM_FWUP_VERIFY_ERROR_IMAGE_INCOMPLETE;
	fprintf(stderr,
		"[pldm-sim] Unknown verify_result: %s, using SUCCESS\n",
		s.c_str());
	return PLDM_FWUP_VERIFY_SUCCESS;
}

static uint8_t parseApplyResult(const std::string &s)
{
	if (s == "SUCCESS")
		return PLDM_FWUP_APPLY_SUCCESS;
	if (s == "SUCCESS_WITH_ACTIVATION_METHOD")
		return PLDM_FWUP_APPLY_SUCCESS_WITH_ACTIVATION_METHOD;
	if (s == "FAILURE_MEMORY_ISSUE")
		return PLDM_FWUP_APPLY_FAILURE_MEMORY_ISSUE;
	fprintf(stderr,
		"[pldm-sim] Unknown apply_result: %s, using SUCCESS\n",
		s.c_str());
	return PLDM_FWUP_APPLY_SUCCESS;
}

/* ===== Scenario loading ===== */

static void loadDefaults(SimConfig &cfg)
{
	/* Original mock_fd behavior: two components, one rejects, one accepts */
	SimComponent reject;
	reject.id = 100;
	reject.classification = 10;
	reject.version = "RejectMe1.0";
	reject.canUpdate = false;
	reject.rejectReason = PLDM_CRC_COMP_NOT_SUPPORTED;

	SimComponent accept;
	accept.id = 200;
	accept.classification = 10;
	accept.version = "AcceptMe1.0";
	accept.canUpdate = true;

	cfg.components.push_back(reject);
	cfg.components.push_back(accept);
}

static SimConfig loadScenario(const std::string &path)
{
	std::ifstream f(path);
	if (!f.is_open()) {
		fprintf(stderr, "[pldm-sim] Cannot open config: %s\n",
			path.c_str());
		exit(1);
	}

	auto data = Json::parse(f, nullptr, false);
	if (data.is_discarded()) {
		fprintf(stderr, "[pldm-sim] Invalid JSON in: %s\n",
			path.c_str());
		exit(1);
	}

	SimConfig cfg;

	if (data.contains("device")) {
		auto &dev = data["device"];
		cfg.eid = dev.value("eid", cfg.eid);
		cfg.tid = dev.value("tid", cfg.tid);
		cfg.name = dev.value("name", cfg.name);
		cfg.entityType = dev.value("entity_type", cfg.entityType);
		cfg.maxTransferSize =
			dev.value("max_transfer_size", cfg.maxTransferSize);

		if (dev.contains("uuid")) {
			if (!parseUuid(dev["uuid"].get<std::string>(),
				       cfg.uuid)) {
				fprintf(stderr,
					"[pldm-sim] Invalid UUID format\n");
				exit(1);
			}
		}
		if (dev.contains("pldm_version")) {
			if (!parsePldmVersion(
				    dev["pldm_version"].get<std::string>(),
				    cfg.pldmVersion)) {
				fprintf(stderr,
					"[pldm-sim] Invalid pldm_version "
					"format (use M.m.u)\n");
				exit(1);
			}
		}
	}

	if (data.contains("firmware_update")) {
		auto &fw = data["firmware_update"];
		cfg.imageSetVersion =
			fw.value("image_set_version", cfg.imageSetVersion);

		if (fw.contains("components")) {
			for (auto &jc : fw["components"]) {
				SimComponent sc;
				sc.id = jc.value("id", sc.id);
				sc.classification =
					jc.value("classification",
						 sc.classification);
				sc.version = jc.value("version", sc.version);
				sc.canUpdate =
					jc.value("can_update", sc.canUpdate);
				sc.activateSelf = jc.value("activate_self",
							   sc.activateSelf);

				if (jc.contains("reject_reason")) {
					sc.rejectReason = parseRejectReason(
						jc["reject_reason"]
							.get<std::string>());
				}
				if (jc.contains("verify_result")) {
					sc.verifyResult = parseVerifyResult(
						jc["verify_result"]
							.get<std::string>());
				}
				if (jc.contains("apply_result")) {
					sc.applyResult = parseApplyResult(
						jc["apply_result"]
							.get<std::string>());
				}

				cfg.components.push_back(sc);
			}
		}
	}

	if (data.contains("platform")) {
		auto &plat = data["platform"];
		cfg.platformEnabled =
			plat.value("enabled", cfg.platformEnabled);
	}

	if (data.contains("timeouts")) {
		auto &t = data["timeouts"];
		cfg.idleTimeoutMs =
			t.value("idle_timeout_ms", cfg.idleTimeoutMs);
		cfg.retryTimeMs = t.value("retry_time_ms", cfg.retryTimeMs);
	}

	/* If no components were specified, use defaults */
	if (cfg.components.empty()) {
		loadDefaults(cfg);
	}

	return cfg;
}

/* ===== Component table building ===== */

static void buildComponentTable(SimContext &ctx)
{
	ctx.compStandalone.clear();
	ctx.compPtrs.clear();

	for (const auto &sc : ctx.cfg.components) {
		pldm_firmware_component_standalone comp = {};
		comp.comp_classification = sc.classification;
		comp.comp_identifier = sc.id;
		comp.comp_classification_index = 0;

		comp.active_ver.comparison_stamp = 0;
		comp.active_ver.str.str_type = PLDM_STR_TYPE_ASCII;
		auto len = std::min(sc.version.size(),
				    static_cast<size_t>(PLDM_FIRMWARE_MAX_STRING));
		comp.active_ver.str.str_len = static_cast<uint8_t>(len);
		memcpy(comp.active_ver.str.str_data, sc.version.c_str(), len);

		comp.pending_ver.str.str_type = PLDM_STR_TYPE_UNKNOWN;
		comp.pending_ver.str.str_len = 0;

		ctx.compStandalone.push_back(comp);
	}

	for (auto &c : ctx.compStandalone) {
		ctx.compPtrs.push_back(&c);
	}
}

/* ===== pldm_fd_ops callbacks ===== */

static int cb_device_identifiers(void *ctx, uint8_t *ret_descriptors_count,
				 const struct pldm_descriptor **ret_descriptors)
{
	auto *sctx = static_cast<SimContext *>(ctx);
	static struct pldm_descriptor desc;
	desc.descriptor_type = PLDM_FWUP_UUID;
	desc.descriptor_length = 16;
	desc.descriptor_data = sctx->cfg.uuid.data();

	*ret_descriptors_count = 1;
	*ret_descriptors = &desc;
	return 0;
}

static int
cb_components(void *ctx, uint16_t *ret_entry_count,
	      const struct pldm_firmware_component_standalone ***ret_entries)
{
	auto *sctx = static_cast<SimContext *>(ctx);
	*ret_entry_count =
		static_cast<uint16_t>(sctx->compPtrs.size());
	*ret_entries = sctx->compPtrs.data();
	return 0;
}

static int cb_imageset_versions(void *ctx,
				struct pldm_firmware_string *ret_active,
				struct pldm_firmware_string *ret_pending)
{
	auto *sctx = static_cast<SimContext *>(ctx);
	ret_active->str_type = PLDM_STR_TYPE_ASCII;
	auto len = std::min(sctx->cfg.imageSetVersion.size(),
			    static_cast<size_t>(PLDM_FIRMWARE_MAX_STRING));
	ret_active->str_len = static_cast<uint8_t>(len);
	memcpy(ret_active->str_data, sctx->cfg.imageSetVersion.c_str(), len);

	ret_pending->str_type = PLDM_STR_TYPE_UNKNOWN;
	ret_pending->str_len = 0;
	return 0;
}

static const SimComponent *findComponent(const SimContext *sctx, uint16_t id)
{
	for (const auto &sc : sctx->cfg.components) {
		if (sc.id == id)
			return &sc;
	}
	return nullptr;
}

static enum pldm_component_response_codes
cb_update_component(void *ctx, bool update,
		    const struct pldm_firmware_update_component *comp)
{
	auto *sctx = static_cast<SimContext *>(ctx);
	const SimComponent *sc = findComponent(sctx, comp->comp_identifier);

	if (update) {
		if (sc && !sc->canUpdate) {
			printf("[pldm-sim] UpdateComponent for comp %u "
			       "(classification %u): CANNOT BE UPDATED\n",
			       comp->comp_identifier,
			       comp->comp_classification);
			return static_cast<enum pldm_component_response_codes>(
				sc->rejectReason);
		}
		printf("[pldm-sim] UpdateComponent for comp %u: "
		       "CAN BE UPDATED\n",
		       comp->comp_identifier);
		sctx->fw_data_received = 0;
	} else {
		printf("[pldm-sim] PassComponentTable for comp %u: "
		       "CAN BE UPDATED\n",
		       comp->comp_identifier);
	}
	return PLDM_CRC_COMP_CAN_BE_UPDATED;
}

static uint32_t cb_transfer_size(void *ctx, uint32_t ua_max_transfer_size)
{
	auto *sctx = static_cast<SimContext *>(ctx);
	uint32_t size = sctx->cfg.maxTransferSize;
	if (size > ua_max_transfer_size) {
		size = ua_max_transfer_size;
	}
	if (size < 32) {
		size = 32;
	}
	return size;
}

static uint8_t cb_firmware_data(void *ctx, uint32_t offset, const uint8_t *data,
				uint32_t len,
				const struct pldm_firmware_update_component *comp)
{
	(void)data;
	auto *sctx = static_cast<SimContext *>(ctx);
	sctx->fw_data_received += len;
	printf("[pldm-sim] FirmwareData: comp %u, offset %u, len %u "
	       "(total %u)\n",
	       comp->comp_identifier, offset, len, sctx->fw_data_received);
	return PLDM_FWUP_TRANSFER_SUCCESS;
}

static uint8_t cb_verify(void *ctx,
			  const struct pldm_firmware_update_component *comp,
			  bool *ret_pending, uint8_t *ret_progress_percent)
{
	(void)ret_progress_percent;
	auto *sctx = static_cast<SimContext *>(ctx);
	const SimComponent *sc = findComponent(sctx, comp->comp_identifier);
	uint8_t result = PLDM_FWUP_VERIFY_SUCCESS;
	if (sc) {
		result = sc->verifyResult;
	}
	printf("[pldm-sim] Verify: comp %u → %s\n", comp->comp_identifier,
	       result == PLDM_FWUP_VERIFY_SUCCESS ? "SUCCESS" : "FAILURE");
	*ret_pending = false;
	return result;
}

static uint8_t cb_apply(void *ctx,
			 const struct pldm_firmware_update_component *comp,
			 bool *ret_pending, uint8_t *ret_progress_percent)
{
	(void)ret_progress_percent;
	auto *sctx = static_cast<SimContext *>(ctx);
	const SimComponent *sc = findComponent(sctx, comp->comp_identifier);
	uint8_t result = PLDM_FWUP_APPLY_SUCCESS;
	if (sc) {
		result = sc->applyResult;
	}
	printf("[pldm-sim] Apply: comp %u → %s\n", comp->comp_identifier,
	       result == PLDM_FWUP_APPLY_SUCCESS ? "SUCCESS" : "FAILURE");
	*ret_pending = false;
	return result;
}

static uint8_t cb_activate(void *ctx, bool self_contained,
			    uint16_t *ret_estimated_time)
{
	(void)ctx;
	printf("[pldm-sim] Activate: self_contained=%d → SUCCESS\n",
	       self_contained);
	*ret_estimated_time = 0;
	return PLDM_SUCCESS;
}

static void
cb_cancel_update_component(void *ctx,
			   const struct pldm_firmware_update_component *comp)
{
	(void)ctx;
	printf("[pldm-sim] CancelUpdateComponent: comp %u\n",
	       comp->comp_identifier);
}

static uint64_t cb_now(void *ctx)
{
	(void)ctx;
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<uint64_t>(ts.tv_sec) * 1000 +
	       static_cast<uint64_t>(ts.tv_nsec) / 1000000;
}

static const struct pldm_fd_ops sim_fd_ops = {
	.device_identifiers = cb_device_identifiers,
	.components = cb_components,
	.imageset_versions = cb_imageset_versions,
	.update_component = cb_update_component,
	.transfer_size = cb_transfer_size,
	.firmware_data = cb_firmware_data,
	.verify = cb_verify,
	.apply = cb_apply,
	.activate = cb_activate,
	.cancel_update_component = cb_cancel_update_component,
	.now = cb_now,
};

/* ===== MCTP control message handling ===== */

static int handle_mctp_control(const SimConfig &cfg, const uint8_t *req,
			       size_t req_len, uint8_t *resp, size_t *resp_len)
{
	if (req_len < 3) {
		return -1;
	}

	uint8_t rq_flags = req[1];
	uint8_t cmd = req[2];
	uint8_t instance = rq_flags & 0x1F;

	if (!(rq_flags & 0x80)) {
		return -1;
	}

	resp[0] = 0x00;
	resp[1] = instance;
	resp[2] = cmd;

	switch (cmd) {
	case MCTP_CTRL_SET_EID:
		printf("[pldm-sim] MCTP Control: Set Endpoint ID\n");
		resp[3] = 0x00;
		resp[4] = 0x00;
		resp[5] = cfg.eid;
		resp[6] = 0x00;
		*resp_len = 7;
		break;

	case MCTP_CTRL_GET_EID:
		printf("[pldm-sim] MCTP Control: Get Endpoint ID\n");
		resp[3] = 0x00;
		resp[4] = cfg.eid;
		resp[5] = 0x00;
		resp[6] = 0x00;
		*resp_len = 7;
		break;

	case MCTP_CTRL_GET_UUID:
		printf("[pldm-sim] MCTP Control: Get Endpoint UUID\n");
		resp[3] = 0x00;
		memcpy(&resp[4], cfg.uuid.data(), 16);
		*resp_len = 20;
		break;

	case MCTP_CTRL_GET_MSG_TYPE:
		printf("[pldm-sim] MCTP Control: Get Message Type Support\n");
		resp[3] = 0x00;
		resp[4] = 2;
		resp[5] = 0x00;
		resp[6] = 0x01;
		*resp_len = 7;
		break;

	default:
		printf("[pldm-sim] MCTP Control: Unknown command 0x%02x\n",
		       cmd);
		resp[3] = 0x05;
		*resp_len = 4;
		break;
	}

	return 0;
}

/* ===== MCTP serial framing for direct TTY write ===== */

static uint16_t crc_ccitt_byte(uint16_t crc, uint8_t c)
{
	crc ^= c;
	for (int i = 0; i < 8; i++) {
		if (crc & 1)
			crc = (crc >> 1) ^ 0x8408;
		else
			crc >>= 1;
	}
	return crc;
}

static uint16_t crc_ccitt(uint16_t crc, const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
		crc = crc_ccitt_byte(crc, data[i]);
	return crc;
}

static constexpr uint16_t MCTP_SERIAL_FCS_INIT = 0xFFFF;
static constexpr uint8_t MCTP_SERIAL_VERSION_BYTE = 0x01;

static int send_mctp_serial_frame(int tty_fd, const uint8_t *data,
				  size_t data_len)
{
	uint8_t frame[512];
	size_t pos = 0;

	if (data_len > 255 || data_len == 0)
		return -1;

	frame[pos++] = 0x7E;
	frame[pos++] = MCTP_SERIAL_VERSION_BYTE;
	frame[pos++] = static_cast<uint8_t>(data_len);

	uint16_t fcs = MCTP_SERIAL_FCS_INIT;
	fcs = crc_ccitt_byte(fcs, MCTP_SERIAL_VERSION_BYTE);
	fcs = crc_ccitt_byte(fcs, static_cast<uint8_t>(data_len));
	fcs = crc_ccitt(fcs, data, data_len);

	for (size_t i = 0; i < data_len; i++) {
		if (data[i] == 0x7E || data[i] == 0x7D) {
			frame[pos++] = 0x7D;
			frame[pos++] = data[i] & ~0x20;
		} else {
			frame[pos++] = data[i];
		}
	}

	frame[pos++] = static_cast<uint8_t>(fcs >> 8);
	frame[pos++] = static_cast<uint8_t>(fcs & 0xFF);
	frame[pos++] = 0x7E;

	ssize_t written = write(tty_fd, frame, pos);
	if (written < 0) {
		perror("[pldm-sim] write tty");
		return -1;
	}
	printf("[pldm-sim] TTY: wrote %zd/%zu byte serial frame "
	       "(%zu byte MCTP packet)\n",
	       written, pos, data_len);
	return 0;
}

/* ===== AF_PACKET raw MCTP handling ===== */

static int create_raw_mctp_socket(const char *ifname, int *out_ifindex)
{
	int sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (sock < 0) {
		perror("socket AF_PACKET");
		return -1;
	}

	unsigned int ifindex = if_nametoindex(ifname);
	if (ifindex == 0) {
		fprintf(stderr, "if_nametoindex(%s): %s\n", ifname,
			strerror(errno));
		close(sock);
		return -1;
	}

	struct sockaddr_ll sll = {};
	sll.sll_family = AF_PACKET;
	sll.sll_protocol = htons(ETH_P_ALL);
	sll.sll_ifindex = static_cast<int>(ifindex);

	if (bind(sock, reinterpret_cast<struct sockaddr *>(&sll),
		 sizeof(sll)) < 0) {
		perror("bind AF_PACKET");
		close(sock);
		return -1;
	}

	*out_ifindex = static_cast<int>(ifindex);
	printf("[pldm-sim] AF_PACKET socket bound to %s (ifindex %d)\n",
	       ifname, static_cast<int>(ifindex));
	return sock;
}

static void handle_raw_mctp_frame(const SimConfig &cfg, int raw_sock,
				  int ctrl_sock, int ifindex,
				  const uint8_t *frame, size_t frame_len)
{
	if (frame_len < 5) {
		printf("[pldm-sim] Raw: frame too short (%zu bytes)\n",
		       frame_len);
		return;
	}

	uint8_t version = frame[0];
	uint8_t dest_eid = frame[1];
	uint8_t src_eid = frame[2];
	uint8_t flags_tag = frame[3];
	uint8_t som = (flags_tag & MCTP_HDR_FLAG_SOM) != 0;
	uint8_t eom = (flags_tag & MCTP_HDR_FLAG_EOM) != 0;
	uint8_t to = (flags_tag & MCTP_HDR_FLAG_TO) != 0;
	uint8_t tag = flags_tag & MCTP_HDR_TAG_MASK;
	uint8_t msg_type = frame[4];

	printf("[pldm-sim] Raw MCTP: ver=%u src=%u dst=%u "
	       "SOM=%u EOM=%u TO=%u tag=%u type=0x%02x len=%zu\n",
	       version, src_eid, dest_eid, som, eom, to, tag, msg_type,
	       frame_len);

	if (msg_type != MCTP_TYPE_CONTROL || !som || !eom) {
		return;
	}

	if (frame_len > 5 && !(frame[5] & 0x80)) {
		return;
	}

	const uint8_t *payload = frame + 4;
	size_t payload_len = frame_len - 4;

	uint8_t resp_payload[256];
	size_t resp_payload_len = sizeof(resp_payload);

	int rc = handle_mctp_control(cfg, payload, payload_len, resp_payload,
				     &resp_payload_len);
	if (rc != 0) {
		return;
	}

	uint8_t resp_frame[260];
	resp_frame[0] = MCTP_HDR_VERSION;
	resp_frame[1] = src_eid;
	resp_frame[2] = cfg.eid;
	resp_frame[3] = MCTP_HDR_FLAG_SOM | MCTP_HDR_FLAG_EOM | tag;
	memcpy(resp_frame + 4, resp_payload, resp_payload_len);

	size_t resp_frame_len = 4 + resp_payload_len;

	struct sockaddr_ll dst = {};
	dst.sll_family = AF_PACKET;
	dst.sll_protocol = htons(ETH_P_MCTP);
	dst.sll_ifindex = ifindex;

	ssize_t sent = sendto(raw_sock, resp_frame, resp_frame_len, 0,
			      reinterpret_cast<struct sockaddr *>(&dst),
			      sizeof(dst));
	if (sent < 0) {
		perror("[pldm-sim] sendto AF_PACKET");
	} else {
		printf("[pldm-sim] Sent %zd byte control response to EID %u\n",
		       sent, src_eid);
	}
}

/* ===== PLDM base message handling ===== */

static int handle_pldm_base(const SimConfig &cfg, const uint8_t *recv_buf,
			     size_t recv_len, uint8_t *send_buf,
			     size_t *send_len)
{
	auto *hdr = reinterpret_cast<const struct pldm_msg_hdr *>(recv_buf);
	auto *req = reinterpret_cast<const struct pldm_msg *>(recv_buf);
	auto *resp = reinterpret_cast<struct pldm_msg *>(send_buf);
	size_t req_payload_len = recv_len - sizeof(struct pldm_msg_hdr);

	printf("[pldm-sim] PLDM Base: command 0x%02x, instance %u\n",
	       hdr->command, hdr->instance_id);

	switch (hdr->command) {
	case PLDM_GET_TID: {
		printf("[pldm-sim]   -> GetTID: responding TID=%u\n", cfg.tid);
		int rc = encode_get_tid_resp(hdr->instance_id, PLDM_SUCCESS,
					     cfg.tid, resp);
		if (rc != PLDM_SUCCESS)
			return -1;
		*send_len = sizeof(struct pldm_msg_hdr) +
			    PLDM_GET_TID_RESP_BYTES;
		return 0;
	}

	case PLDM_SET_TID: {
		uint8_t tid = 0;
		if (req_payload_len >= 1) {
			decode_set_tid_req(req, req_payload_len, &tid);
		}
		printf("[pldm-sim]   -> SetTID: accepting TID=%u\n", tid);
		struct pldm_header_info hdr_info = {};
		hdr_info.msg_type = PLDM_RESPONSE;
		hdr_info.instance = hdr->instance_id;
		hdr_info.pldm_type = PLDM_BASE;
		hdr_info.command = PLDM_SET_TID;
		pack_pldm_header(&hdr_info, &resp->hdr);
		resp->payload[0] = PLDM_SUCCESS;
		*send_len = sizeof(struct pldm_msg_hdr) +
			    PLDM_SET_TID_RESP_BYTES;
		return 0;
	}

	case PLDM_GET_PLDM_TYPES: {
		bitfield8_t types[8] = {};
		types[0].byte = (1 << PLDM_BASE) | (1 << PLDM_FWUP);
		if (cfg.platformEnabled) {
			types[0].byte |= (1 << PLDM_PLATFORM);
		}
		printf("[pldm-sim]   -> GetPLDMTypes: BASE + FWUP%s\n",
		       cfg.platformEnabled ? " + PLATFORM" : "");
		int rc = encode_get_types_resp(hdr->instance_id, PLDM_SUCCESS,
					       types, resp);
		if (rc != PLDM_SUCCESS)
			return -1;
		*send_len = sizeof(struct pldm_msg_hdr) +
			    PLDM_GET_TYPES_RESP_BYTES;
		return 0;
	}

	case PLDM_GET_PLDM_VERSION: {
		printf("[pldm-sim]   -> GetPLDMVersion\n");
		ver32_t version = {};
		version.alpha = 0x00;
		version.update = cfg.pldmVersion[2];
		version.minor = cfg.pldmVersion[1];
		version.major = cfg.pldmVersion[0];
		int rc = encode_get_version_resp(
			hdr->instance_id, PLDM_SUCCESS, 0,
			PLDM_START_AND_END, &version, sizeof(version), resp);
		if (rc != PLDM_SUCCESS)
			return -1;
		*send_len = sizeof(struct pldm_msg_hdr) +
			    PLDM_GET_VERSION_RESP_BYTES;
		return 0;
	}

	case PLDM_GET_PLDM_COMMANDS: {
		uint8_t req_type = 0;
		ver32_t req_version = {};
		if (req_payload_len >= 5) {
			decode_get_commands_req(req, req_payload_len, &req_type,
					       &req_version);
		}
		printf("[pldm-sim]   -> GetPLDMCommands for type %u\n",
		       req_type);

		bitfield8_t commands[32] = {};
		if (req_type == PLDM_BASE) {
			commands[0].byte = (1 << PLDM_SET_TID) |
					   (1 << PLDM_GET_TID) |
					   (1 << PLDM_GET_PLDM_VERSION) |
					   (1 << PLDM_GET_PLDM_TYPES) |
					   (1 << PLDM_GET_PLDM_COMMANDS);
		} else if (req_type == PLDM_PLATFORM &&
			   cfg.platformEnabled) {
			commands[10].byte = (1 << (PLDM_GET_PDR & 7));
		} else if (req_type == PLDM_FWUP) {
			commands[0].byte =
				(1 << PLDM_QUERY_DEVICE_IDENTIFIERS) |
				(1 << PLDM_GET_FIRMWARE_PARAMETERS);
			commands[2].byte =
				(1 << (PLDM_REQUEST_UPDATE & 7)) |
				(1 << (PLDM_PASS_COMPONENT_TABLE & 7)) |
				(1 << (PLDM_UPDATE_COMPONENT & 7)) |
				(1 << (PLDM_REQUEST_FIRMWARE_DATA & 7)) |
				(1 << (PLDM_TRANSFER_COMPLETE & 7)) |
				(1 << (PLDM_VERIFY_COMPLETE & 7));
			commands[3].byte =
				(1 << (PLDM_APPLY_COMPLETE & 7)) |
				(1 << (PLDM_ACTIVATE_FIRMWARE & 7)) |
				(1 << (PLDM_GET_STATUS & 7)) |
				(1 << (PLDM_CANCEL_UPDATE_COMPONENT & 7)) |
				(1 << (PLDM_CANCEL_UPDATE & 7));
		}

		int rc = encode_get_commands_resp(hdr->instance_id,
						  PLDM_SUCCESS, commands, resp);
		if (rc != PLDM_SUCCESS)
			return -1;
		*send_len = sizeof(struct pldm_msg_hdr) +
			    PLDM_GET_COMMANDS_RESP_BYTES;
		return 0;
	}

	default:
		printf("[pldm-sim]   -> Unsupported base command 0x%02x\n",
		       hdr->command);
		{
			struct pldm_header_info hdr_info = {};
			hdr_info.msg_type = PLDM_RESPONSE;
			hdr_info.instance = hdr->instance_id;
			hdr_info.pldm_type = PLDM_BASE;
			hdr_info.command = hdr->command;
			pack_pldm_header(&hdr_info, &resp->hdr);
			resp->payload[0] = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;
			*send_len = sizeof(struct pldm_msg_hdr) + 1;
		}
		return 0;
	}
}

/* ===== PLDM Platform Monitoring & Control ===== */

static size_t build_terminus_locator_pdr(uint8_t *buf, size_t buf_len,
					 uint32_t record_handle,
					 uint8_t eid, uint8_t tid)
{
	const size_t data_len = 2 + 1 + 1 + 2 + 1 + 1 + 1;
	const size_t total = sizeof(struct pldm_pdr_hdr) + data_len;

	if (buf_len < total)
		return 0;

	memset(buf, 0, total);

	buf[0] = record_handle & 0xFF;
	buf[1] = (record_handle >> 8) & 0xFF;
	buf[2] = (record_handle >> 16) & 0xFF;
	buf[3] = (record_handle >> 24) & 0xFF;
	buf[4] = 0x01;
	buf[5] = PLDM_TERMINUS_LOCATOR_PDR;
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = data_len & 0xFF;
	buf[9] = (data_len >> 8) & 0xFF;

	uint8_t *d = buf + sizeof(struct pldm_pdr_hdr);
	d[0] = 0x01;
	d[1] = 0x00;
	d[2] = 0x01;
	d[3] = tid;
	d[4] = 0x00;
	d[5] = 0x00;
	d[6] = 0x01;
	d[7] = 0x01;
	d[8] = eid;

	return total;
}

static size_t build_entity_aux_names_pdr(uint8_t *buf, size_t buf_len,
					 uint32_t record_handle,
					 uint16_t entity_type,
					 const char *name)
{
	size_t name_len = strlen(name);

	/* Tag "en" + NUL = 3 bytes; name in UTF-16BE + NUL = name_len*2+2 */
	size_t tag_len = 3; /* "en\0" */
	size_t u16_name_bytes = name_len * 2 + 2;
	/* Fixed fields: entityType(2) + instanceNum(2) + containerID(2) +
	 * sharedNameCount(1) + nameStringCount(1) = 8 */
	size_t data_len = 8 + tag_len + u16_name_bytes;
	size_t total = sizeof(struct pldm_pdr_hdr) + data_len;

	if (buf_len < total)
		return 0;

	memset(buf, 0, total);

	buf[0] = record_handle & 0xFF;
	buf[1] = (record_handle >> 8) & 0xFF;
	buf[2] = (record_handle >> 16) & 0xFF;
	buf[3] = (record_handle >> 24) & 0xFF;
	buf[4] = 0x01;
	buf[5] = PLDM_ENTITY_AUXILIARY_NAMES_PDR;
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = data_len & 0xFF;
	buf[9] = (data_len >> 8) & 0xFF;

	uint8_t *d = buf + sizeof(struct pldm_pdr_hdr);
	d[0] = entity_type & 0xFF;
	d[1] = (entity_type >> 8) & 0xFF;
	d[2] = 0x01; /* entityInstanceNumber = 1 */
	d[3] = 0x00;
	d[4] = 0x00; /* entityContainerID = 0 */
	d[5] = 0x00;
	d[6] = 0x00; /* sharedNameCount = 0 */
	d[7] = 0x01; /* nameStringCount = 1 */
	/* Per DSP0248 Table 95: nameLanguageTag (ASCII) then
	 * entityAuxiliaryName (UTF-16BE) for each entry */
	d[8] = 'e';
	d[9] = 'n';
	d[10] = 0x00; /* tag NUL terminator */

	uint8_t *np = d + 11; /* UTF-16BE name follows immediately */
	for (size_t i = 0; i < name_len; i++) {
		np[i * 2] = 0x00;
		np[i * 2 + 1] = static_cast<uint8_t>(name[i]);
	}
	np[name_len * 2] = 0x00;
	np[name_len * 2 + 1] = 0x00;

	return total;
}

static void init_pdr_repository(SimContext &ctx)
{
	ctx.pdrRecordSizes[0] = build_terminus_locator_pdr(
		ctx.pdrRecords[0], sizeof(ctx.pdrRecords[0]), 1, ctx.cfg.eid,
		ctx.cfg.tid);
	ctx.pdrRecordSizes[1] = build_entity_aux_names_pdr(
		ctx.pdrRecords[1], sizeof(ctx.pdrRecords[1]), 2,
		ctx.cfg.entityType, ctx.cfg.name.c_str());
	ctx.pdrRecordCount = 2;

	printf("[pldm-sim] PDR repository: %d records "
	       "(entity_type=%u, name=%s)\n",
	       ctx.pdrRecordCount, ctx.cfg.entityType, ctx.cfg.name.c_str());
}

static int handle_pldm_platform(SimContext &ctx, const uint8_t *recv_buf,
				size_t recv_len, uint8_t *send_buf,
				size_t *send_len)
{
	auto *hdr = reinterpret_cast<const struct pldm_msg_hdr *>(recv_buf);
	auto *req = reinterpret_cast<const struct pldm_msg *>(recv_buf);
	auto *resp = reinterpret_cast<struct pldm_msg *>(send_buf);
	size_t req_payload_len = recv_len - sizeof(struct pldm_msg_hdr);

	printf("[pldm-sim] PLDM Platform: command 0x%02x, instance %u\n",
	       hdr->command, hdr->instance_id);

	if (hdr->command != PLDM_GET_PDR) {
		printf("[pldm-sim]   -> Unsupported platform command 0x%02x\n",
		       hdr->command);
		struct pldm_header_info hdr_info = {};
		hdr_info.msg_type = PLDM_RESPONSE;
		hdr_info.instance = hdr->instance_id;
		hdr_info.pldm_type = PLDM_PLATFORM;
		hdr_info.command = hdr->command;
		pack_pldm_header(&hdr_info, &resp->hdr);
		resp->payload[0] = PLDM_ERROR_UNSUPPORTED_PLDM_CMD;
		*send_len = sizeof(struct pldm_msg_hdr) + 1;
		return 0;
	}

	uint32_t record_hndl = 0;
	uint32_t data_transfer_hndl = 0;
	uint8_t transfer_op_flag = 0;
	uint16_t request_cnt = 0;
	uint16_t record_chg_num = 0;

	int rc = decode_get_pdr_req(req, req_payload_len, &record_hndl,
				    &data_transfer_hndl, &transfer_op_flag,
				    &request_cnt, &record_chg_num);
	if (rc != PLDM_SUCCESS) {
		printf("[pldm-sim]   -> Failed to decode GetPDR request: %d\n",
		       rc);
		struct pldm_header_info hdr_info = {};
		hdr_info.msg_type = PLDM_RESPONSE;
		hdr_info.instance = hdr->instance_id;
		hdr_info.pldm_type = PLDM_PLATFORM;
		hdr_info.command = PLDM_GET_PDR;
		pack_pldm_header(&hdr_info, &resp->hdr);
		resp->payload[0] = PLDM_ERROR;
		*send_len = sizeof(struct pldm_msg_hdr) + 1;
		return 0;
	}

	printf("[pldm-sim]   -> GetPDR: record_handle=%u, request_cnt=%u\n",
	       record_hndl, request_cnt);

	int idx;
	if (record_hndl == 0) {
		idx = 0;
	} else {
		idx = static_cast<int>(record_hndl) - 1;
	}

	struct pldm_header_info hdr_info = {};
	hdr_info.msg_type = PLDM_RESPONSE;
	hdr_info.instance = hdr->instance_id;
	hdr_info.pldm_type = PLDM_PLATFORM;
	hdr_info.command = PLDM_GET_PDR;
	pack_pldm_header(&hdr_info, &resp->hdr);
	uint8_t *p = resp->payload;

	if (idx < 0 || idx >= ctx.pdrRecordCount) {
		printf("[pldm-sim]   -> Record not found\n");
		p[0] = PLDM_PLATFORM_INVALID_RECORD_HANDLE;
		memset(p + 1, 0, 11);
		*send_len = sizeof(struct pldm_msg_hdr) +
			    PLDM_GET_PDR_MIN_RESP_BYTES;
		return 0;
	}

	uint32_t next_hndl = 0;
	if (idx + 1 < ctx.pdrRecordCount) {
		next_hndl = static_cast<uint32_t>(idx + 2);
	}

	auto resp_cnt =
		static_cast<uint16_t>(ctx.pdrRecordSizes[idx]);
	printf("[pldm-sim]   -> Returning record %d (%u bytes), "
	       "next_handle=%u\n",
	       idx + 1, resp_cnt, next_hndl);

	p[0] = PLDM_SUCCESS;
	p[1] = next_hndl & 0xFF;
	p[2] = (next_hndl >> 8) & 0xFF;
	p[3] = (next_hndl >> 16) & 0xFF;
	p[4] = (next_hndl >> 24) & 0xFF;
	p[5] = 0;
	p[6] = 0;
	p[7] = 0;
	p[8] = 0;
	p[9] = PLDM_START_AND_END;
	p[10] = resp_cnt & 0xFF;
	p[11] = (resp_cnt >> 8) & 0xFF;
	memcpy(p + 12, ctx.pdrRecords[idx], resp_cnt);

	*send_len = sizeof(struct pldm_msg_hdr) + 12 + resp_cnt;
	return 0;
}

/* ===== TTY and MCTP setup ===== */

static int setup_tty(const char *pty_path)
{
	int fd = open(pty_path, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror("open PTY");
		return -1;
	}

	int ldisc = N_MCTP;
	if (ioctl(fd, TIOCSETD, &ldisc) < 0) {
		perror("ioctl TIOCSETD N_MCTP");
		fprintf(stderr,
			"Is mctp-serial module loaded? "
			"Try: modprobe mctp-serial\n");
		close(fd);
		return -1;
	}

	printf("[pldm-sim] MCTP serial line discipline set on %s\n", pty_path);
	return fd;
}

static int find_mctp_interface(char *ifname, size_t ifname_len)
{
	FILE *fp = popen("ls -1 /sys/class/net/ | grep '^mctp' | tail -1",
			 "r");
	if (!fp) {
		return -1;
	}
	if (fgets(ifname, static_cast<int>(ifname_len), fp) == NULL) {
		pclose(fp);
		return -1;
	}
	pclose(fp);

	size_t len = strlen(ifname);
	if (len > 0 && ifname[len - 1] == '\n') {
		ifname[len - 1] = '\0';
	}

	if (strlen(ifname) == 0) {
		return -1;
	}

	printf("[pldm-sim] Found MCTP interface: %s\n", ifname);
	return 0;
}

static int setup_mctp_addressing(const char *ifname, uint8_t eid)
{
	char cmd[512];
	int rc;

	snprintf(cmd, sizeof(cmd), "%s link set %s up 2>&1", mctp_tool_path,
		 ifname);
	printf("[pldm-sim] Running: %s\n", cmd);
	rc = system(cmd);
	if (rc != 0) {
		fprintf(stderr, "Failed to bring up %s\n", ifname);
		return -1;
	}

	snprintf(cmd, sizeof(cmd), "%s addr add %d dev %s 2>&1",
		 mctp_tool_path, eid, ifname);
	printf("[pldm-sim] Running: %s\n", cmd);
	rc = system(cmd);
	if (rc != 0) {
		printf("[pldm-sim] EID add returned %d "
		       "(may already exist, continuing)\n",
		       rc);
	}

	snprintf(cmd, sizeof(cmd), "%s route add %d via %s 2>&1",
		 mctp_tool_path, BMC_EID, ifname);
	printf("[pldm-sim] Running: %s\n", cmd);
	rc = system(cmd);
	if (rc != 0) {
		printf("[pldm-sim] Route add returned %d "
		       "(may already exist, continuing)\n",
		       rc);
	}

	printf("[pldm-sim] MCTP addressing configured: EID %d, "
	       "route to EID %d\n",
	       eid, BMC_EID);
	return 0;
}

static int create_mctp_socket(uint8_t msg_type, mctp_eid_t bind_eid)
{
	int sock = socket(AF_MCTP, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("socket AF_MCTP");
		return -1;
	}

	struct sockaddr_mctp addr = {};
	addr.smctp_family = AF_MCTP;
	addr.smctp_network = MCTP_NET_ANY;
	addr.smctp_addr.s_addr = bind_eid;
	addr.smctp_type = msg_type;
	addr.smctp_tag = MCTP_TAG_OWNER;

	if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr),
		 sizeof(addr)) < 0) {
		perror("bind AF_MCTP");
		close(sock);
		return -1;
	}

	printf("[pldm-sim] AF_MCTP socket bound: EID %d, type 0x%02x\n",
	       bind_eid, msg_type);
	return sock;
}

static int send_mctp_msg(int sock, uint8_t dest_eid, uint8_t msg_type,
			 const void *data, size_t len)
{
	struct sockaddr_mctp addr = {};
	addr.smctp_family = AF_MCTP;
	addr.smctp_network = MCTP_NET_ANY;
	addr.smctp_addr.s_addr = dest_eid;
	addr.smctp_type = msg_type;
	addr.smctp_tag = MCTP_TAG_OWNER;

	ssize_t rc = sendto(sock, data, len, 0,
			    reinterpret_cast<struct sockaddr *>(&addr),
			    sizeof(addr));
	if (rc < 0) {
		perror("sendto");
		return -1;
	}
	return 0;
}

/* ===== Signal handling ===== */

static void sigint_handler(int sig)
{
	(void)sig;
	running = 0;
}

/* ===== Main ===== */

static void print_usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [options] /dev/pts/N [mctp-interface]\n\n", prog);
	fprintf(stderr,
		"  /dev/pts/N                PTY from QEMU -serial pty\n");
	fprintf(stderr,
		"  mctp-interface            Optional: MCTP interface name "
		"(auto-detected)\n\n");
	fprintf(stderr, "Options:\n");
	fprintf(stderr,
		"  --config <file>           Load scenario from JSON file\n");
	fprintf(stderr,
		"  --help, -h                Print usage and exit\n\n");
	fprintf(stderr,
		"Without --config, uses built-in defaults "
		"(two components: one rejects, one accepts).\n");
}

int main(int argc, char *argv[])
{
	const char *pty_path = nullptr;
	const char *ifname_override = nullptr;
	const char *config_path = nullptr;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--config") == 0) {
			if (i + 1 >= argc) {
				fprintf(stderr,
					"Missing value for --config\n");
				print_usage(argv[0]);
				return 1;
			}
			config_path = argv[++i];
		} else if (strcmp(argv[i], "--help") == 0 ||
			   strcmp(argv[i], "-h") == 0) {
			print_usage(argv[0]);
			return 0;
		} else if (argv[i][0] == '-') {
			fprintf(stderr, "Unknown option: %s\n", argv[i]);
			print_usage(argv[0]);
			return 1;
		} else if (!pty_path) {
			pty_path = argv[i];
		} else if (!ifname_override) {
			ifname_override = argv[i];
		}
	}

	if (!pty_path) {
		print_usage(argv[0]);
		return 1;
	}

	/* Load configuration */
	SimConfig cfg;
	if (config_path) {
		cfg = loadScenario(config_path);
		printf("[pldm-sim] Loaded scenario: %s\n", config_path);
	} else {
		loadDefaults(cfg);
	}

	mctp_tool_path = getenv("MCTP_TOOL");
	if (!mctp_tool_path) {
		mctp_tool_path = "/home/gmbeihl/claude/mctp-host/build/mctp";
	}

	signal(SIGINT, sigint_handler);
	signal(SIGTERM, sigint_handler);

	printf("[pldm-sim] Starting PLDM device simulator\n");
	printf("[pldm-sim] PTY: %s\n", pty_path);
	printf("[pldm-sim] MCTP tool: %s\n", mctp_tool_path);
	printf("[pldm-sim] EID: %u, TID: %u, Name: %s\n", cfg.eid, cfg.tid,
	       cfg.name.c_str());
	printf("[pldm-sim] Components: %zu\n", cfg.components.size());
	for (const auto &sc : cfg.components) {
		printf("[pldm-sim]   comp %u: %s (can_update=%s)\n", sc.id,
		       sc.version.c_str(), sc.canUpdate ? "true" : "false");
	}
	if (cfg.platformEnabled) {
		printf("[pldm-sim] Platform monitoring: entity_type=%u, "
		       "name=%s\n",
		       cfg.entityType, cfg.name.c_str());
	}

	/* Set up TTY with MCTP serial line discipline */
	int tty_fd = setup_tty(pty_path);
	if (tty_fd < 0) {
		return 1;
	}

	/* Find MCTP interface and configure addressing */
	char ifname[64];
	if (ifname_override) {
		snprintf(ifname, sizeof(ifname), "%s", ifname_override);
	} else {
		usleep(500000);
		if (find_mctp_interface(ifname, sizeof(ifname)) < 0) {
			fprintf(stderr,
				"[pldm-sim] No MCTP interface found\n");
			close(tty_fd);
			return 1;
		}
	}

	if (setup_mctp_addressing(ifname, cfg.eid) < 0) {
		close(tty_fd);
		return 1;
	}

	/* Create sockets */
	int pldm_sock = create_mctp_socket(MCTP_TYPE_PLDM, cfg.eid);
	if (pldm_sock < 0) {
		close(tty_fd);
		return 1;
	}

	int ctrl_sock = create_mctp_socket(MCTP_TYPE_CONTROL, cfg.eid);
	if (ctrl_sock < 0) {
		fprintf(stderr,
			"[pldm-sim] Warning: AF_MCTP control socket failed, "
			"falling back to AF_PACKET only\n");
	}

	int ifindex;
	int raw_sock = create_raw_mctp_socket(ifname, &ifindex);
	if (raw_sock < 0) {
		close(pldm_sock);
		if (ctrl_sock >= 0)
			close(ctrl_sock);
		close(tty_fd);
		return 1;
	}

	/* Initialize context */
	SimContext ctx = {};
	ctx.pldm_sock = pldm_sock;
	ctx.ctrl_sock = ctrl_sock;
	ctx.raw_sock = raw_sock;
	ctx.tty_fd = tty_fd;
	ctx.ifindex = ifindex;
	ctx.fw_data_received = 0;
	ctx.cfg = std::move(cfg);
	ctx.pdrRecordCount = 0;

	/* Build dynamic component table */
	buildComponentTable(ctx);

	/* Initialize pldm_fd */
	struct pldm_fd *fd = pldm_fd_new(&sim_fd_ops, &ctx, nullptr);
	if (!fd) {
		fprintf(stderr, "[pldm-sim] Failed to create pldm_fd\n");
		close(raw_sock);
		close(pldm_sock);
		close(tty_fd);
		return 1;
	}
	ctx.fd = fd;

	pldm_fd_set_update_idle_timeout(fd, ctx.cfg.idleTimeoutMs);
	pldm_fd_set_request_retry_time(fd, ctx.cfg.retryTimeMs);

	/* Initialize PDR repository if platform monitoring enabled */
	if (ctx.cfg.platformEnabled) {
		init_pdr_repository(ctx);
	}

	printf("[pldm-sim] pldm_fd initialized, entering event loop\n");
	printf("[pldm-sim] Waiting for %s from pldmd (EID %d)...\n",
	       ctx.cfg.platformEnabled ? "platform discovery" :
					 "firmware update",
	       BMC_EID);

	/* Event loop */
	uint8_t recv_buf[4096];
	uint8_t send_buf[4096];
	struct timespec last_progress;
	clock_gettime(CLOCK_MONOTONIC, &last_progress);

	while (running) {
		struct pollfd pfds[3] = {};
		pfds[0].fd = pldm_sock;
		pfds[0].events = POLLIN;
		pfds[1].fd = raw_sock;
		pfds[1].events = POLLIN;
		pfds[2].fd = ctrl_sock;
		pfds[2].events = 0; /* Disabled: raw handler covers control */
		int nfds = (ctrl_sock >= 0) ? 3 : 2;

		int ret = poll(pfds, nfds, 1000);
		if (ret < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("poll");
			break;
		}

		/* Handle PLDM messages via AF_MCTP */
		if (pfds[0].revents & POLLIN) {
			struct sockaddr_mctp src_addr = {};
			socklen_t addrlen = sizeof(src_addr);
			ssize_t n = recvfrom(
				pldm_sock, recv_buf, sizeof(recv_buf), 0,
				reinterpret_cast<struct sockaddr *>(&src_addr),
				&addrlen);
			if (n >= static_cast<ssize_t>(
					 sizeof(struct pldm_msg_hdr))) {
				auto *hdr =
					reinterpret_cast<
						const struct pldm_msg_hdr *>(
						recv_buf);
				uint8_t pldm_type = hdr->type;

				printf("[pldm-sim] PLDM msg from EID %d, "
				       "%zd bytes, type=%u cmd=0x%02x\n",
				       src_addr.smctp_addr.s_addr, n,
				       pldm_type, hdr->command);

				size_t out_len = sizeof(send_buf);
				int rc;

				if (pldm_type == PLDM_BASE) {
					rc = handle_pldm_base(ctx.cfg, recv_buf,
							      static_cast<size_t>(n),
							      send_buf,
							      &out_len);
				} else if (pldm_type == PLDM_PLATFORM &&
					   ctx.cfg.platformEnabled) {
					rc = handle_pldm_platform(
						ctx, recv_buf,
						static_cast<size_t>(n),
						send_buf, &out_len);
				} else {
					rc = pldm_fd_handle_msg(
						fd,
						src_addr.smctp_addr.s_addr,
						recv_buf,
						static_cast<size_t>(n),
						send_buf, &out_len);
				}

				if (rc == 0 && out_len > 0) {
					struct sockaddr_mctp dst = {};
					dst.smctp_family = AF_MCTP;
					dst.smctp_network = MCTP_NET_ANY;
					dst.smctp_addr.s_addr =
						src_addr.smctp_addr.s_addr;
					dst.smctp_type = MCTP_TYPE_PLDM;
					/* Clear MCTP_TAG_OWNER to send as
					 * response (TO=0), not new request */
					dst.smctp_tag = src_addr.smctp_tag
						& ~MCTP_TAG_OWNER;
					ssize_t sent = sendto(
						pldm_sock, send_buf, out_len, 0,
						reinterpret_cast<
							struct sockaddr *>(
							&dst),
						sizeof(dst));
					if (sent < 0) {
						perror("sendto PLDM response");
					} else {
						printf("[pldm-sim] Sent PLDM "
						       "response, %zd bytes\n",
						       sent);
					}
				} else if (rc != 0) {
					printf("[pldm-sim] PLDM handler "
					       "error: %d\n",
					       rc);
				}
			}
		}

		/* Handle raw MCTP frames via AF_PACKET */
		if (pfds[1].revents & POLLIN) {
			struct sockaddr_ll src_sll = {};
			socklen_t sll_len = sizeof(src_sll);
			ssize_t n = recvfrom(
				raw_sock, recv_buf, sizeof(recv_buf), 0,
				reinterpret_cast<struct sockaddr *>(&src_sll),
				&sll_len);
			if (n > 0) {
				handle_raw_mctp_frame(ctx.cfg, raw_sock,
						      ctrl_sock, ifindex,
						      recv_buf,
						      static_cast<size_t>(n));
			}
		}

		/* Handle MCTP control messages via AF_MCTP */
		if (pfds[2].revents & POLLIN) {
			struct sockaddr_mctp src_addr = {};
			socklen_t addrlen = sizeof(src_addr);
			ssize_t n = recvfrom(
				ctrl_sock, recv_buf, sizeof(recv_buf), 0,
				reinterpret_cast<struct sockaddr *>(&src_addr),
				&addrlen);
			if (n > 0) {
				printf("[pldm-sim] AF_MCTP control: %zd bytes "
				       "from EID %d\n",
				       n, src_addr.smctp_addr.s_addr);

				/*
				 * AF_MCTP strips the message type byte,
				 * delivering only [ctrl_hdr...].
				 * Prepend msg_type=0x00 so handle_mctp_control
				 * gets (req[0]=msg_type, req[1]=flags, req[2]=cmd).
				 */
				uint8_t ctrl_buf[256];
				ctrl_buf[0] = MCTP_TYPE_CONTROL;
				memcpy(ctrl_buf + 1, recv_buf,
				       static_cast<size_t>(n));
				uint8_t resp_payload[256];
				size_t resp_len = sizeof(resp_payload);
				int rc = handle_mctp_control(
					ctx.cfg, ctrl_buf,
					static_cast<size_t>(n) + 1,
					resp_payload, &resp_len);
				if (rc == 0 && resp_len > 0) {
					struct sockaddr_mctp dst = {};
					dst.smctp_family = AF_MCTP;
					dst.smctp_network = MCTP_NET_ANY;
					dst.smctp_addr.s_addr =
						src_addr.smctp_addr.s_addr;
					dst.smctp_type = MCTP_TYPE_CONTROL;
					dst.smctp_tag = src_addr.smctp_tag;
					/*
					 * Strip msg_type from response;
					 * AF_MCTP adds it from smctp_type.
					 */
					ssize_t sent = sendto(
						ctrl_sock,
						resp_payload + 1,
						resp_len - 1, 0,
						reinterpret_cast<
							struct sockaddr *>(
							&dst),
						sizeof(dst));
					if (sent < 0) {
						perror("[pldm-sim] sendto "
						       "ctrl response");
					} else {
						printf("[pldm-sim] Sent ctrl "
						       "response, %zd bytes "
						       "to EID %d\n",
						       sent,
						       src_addr.smctp_addr
							       .s_addr);
					}
				}
			}
		}

		/* Call pldm_fd_progress periodically */
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		long elapsed_ms =
			(now.tv_sec - last_progress.tv_sec) * 1000 +
			(now.tv_nsec - last_progress.tv_nsec) / 1000000;

		if (elapsed_ms >= 1000) {
			last_progress = now;
			size_t out_len = sizeof(send_buf);
			pldm_tid_t dest_addr;
			int rc = pldm_fd_progress(fd, send_buf, &out_len,
						  &dest_addr);
			if (rc == 0 && out_len > 0) {
				printf("[pldm-sim] Progress: sending %zu "
				       "bytes to EID %d\n",
				       out_len, dest_addr);
				send_mctp_msg(pldm_sock, dest_addr,
					      MCTP_TYPE_PLDM, send_buf,
					      out_len);
			}
		}
	}

	printf("[pldm-sim] Shutting down\n");
	free(fd);
	close(raw_sock);
	if (ctrl_sock >= 0)
		close(ctrl_sock);
	close(pldm_sock);
	close(tty_fd);

	return 0;
}
