#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "io/PrtProtocol.h"
#include "model/RadarConfig.h"

namespace
{
    struct VerifyOptions
    {
        std::string input_path = "build/output_first_900MB.dat";
        std::size_t payload_bytes = 1464;
        std::size_t scan_bytes = 32ULL * 1024ULL * 1024ULL;
        std::size_t max_cpi = 0;
        std::size_t preview_cpi = 5;
        std::size_t inspect_prt_index = std::numeric_limits<std::size_t>::max();
        std::size_t inspect_hex_bytes = 32;
    };

    struct CpiSummary
    {
        std::size_t cpi_index = 0;
        std::size_t prt_count = 0;
        uint16_t first_prt_num = 0;
        uint16_t last_prt_num = 0;
        uint32_t global_start = 0;
        uint32_t global_end = 0;
        radar::PrtHeaderInfo first_header{};
    };

    struct VerifyStats
    {
        std::size_t udp_packets = 0;
        std::size_t prt_count = 0;
        std::size_t cpi_count = 0;
        std::size_t resync_drop_bytes = 0;
        std::size_t invalid_head_count = 0;
        std::size_t prt_gap_count = 0;
        std::size_t global_gap_count = 0;
        std::size_t trailing_bytes = 0;
    };

    struct BeamStats
    {
        std::unordered_map<uint32_t, std::size_t> beam_hist;
        std::size_t zero_zero_count = 0;
        std::vector<std::size_t> zero_zero_cpi_indices;
        std::vector<uint32_t> zero_zero_global_starts;
    };

    struct PaddingCheckSummary
    {
        std::size_t expected_signal_bytes = 0;
        std::size_t expected_total_bytes = 0;
        std::size_t expected_padded_span = 0;
        std::size_t expected_padding_bytes = 0;
        bool span_matches = false;
        std::size_t blocks_checked = 0;
        std::size_t bytes_checked = 0;
        std::size_t ff_bytes = 0;
        std::size_t zero_bytes = 0;
        std::size_t other_bytes = 0;
        std::vector<uint8_t> first_block_prefix;
    };

    VerifyOptions parse_options(int argc, char **argv)
    {
        VerifyOptions opt;
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--file" && i + 1 < argc)
            {
                opt.input_path = argv[++i];
            }
            else if (arg.rfind("--file=", 0) == 0)
            {
                opt.input_path = arg.substr(7);
            }
            else if (arg == "--payload-bytes" && i + 1 < argc)
            {
                opt.payload_bytes = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--payload-bytes=", 0) == 0)
            {
                opt.payload_bytes = static_cast<std::size_t>(std::stoull(arg.substr(16)));
            }
            else if (arg == "--scan-bytes" && i + 1 < argc)
            {
                opt.scan_bytes = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--scan-bytes=", 0) == 0)
            {
                opt.scan_bytes = static_cast<std::size_t>(std::stoull(arg.substr(13)));
            }
            else if (arg == "--max-cpi" && i + 1 < argc)
            {
                opt.max_cpi = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--max-cpi=", 0) == 0)
            {
                opt.max_cpi = static_cast<std::size_t>(std::stoull(arg.substr(10)));
            }
            else if (arg == "--preview-cpi" && i + 1 < argc)
            {
                opt.preview_cpi = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--preview-cpi=", 0) == 0)
            {
                opt.preview_cpi = static_cast<std::size_t>(std::stoull(arg.substr(14)));
            }
            else if (arg == "--inspect-prt" && i + 1 < argc)
            {
                opt.inspect_prt_index = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--inspect-prt=", 0) == 0)
            {
                opt.inspect_prt_index = static_cast<std::size_t>(std::stoull(arg.substr(14)));
            }
            else if (arg == "--inspect-bytes" && i + 1 < argc)
            {
                opt.inspect_hex_bytes = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--inspect-bytes=", 0) == 0)
            {
                opt.inspect_hex_bytes = static_cast<std::size_t>(std::stoull(arg.substr(16)));
            }
            else if (arg == "--help" || arg == "-h")
            {
                std::cout << "Usage: ./udp_offline_verify [--file PATH] [--payload-bytes N] [--scan-bytes N]\n";
                std::cout << "                           [--max-cpi N] [--preview-cpi N]\n";
                std::cout << "                           [--inspect-prt N] [--inspect-bytes N]\n";
                std::exit(0);
            }
            else
            {
                throw std::invalid_argument("unknown argument: " + arg);
            }
        }
        return opt;
    }

    std::vector<uint8_t> load_prefix(const std::string &path, std::size_t scan_bytes)
    {
        std::ifstream in(path, std::ios::binary);
        if (!in)
        {
            throw std::runtime_error("failed to open input file: " + path);
        }

        std::vector<uint8_t> data(scan_bytes);
        in.read(reinterpret_cast<char *>(data.data()),
                static_cast<std::streamsize>(data.size()));
        data.resize(static_cast<std::size_t>(in.gcount()));
        return data;
    }

    std::vector<std::size_t> find_prt_offsets(const std::vector<uint8_t> &data)
    {
        static constexpr uint8_t kPrtHeadLE[4] = {0x32, 0xCD, 0x55, 0xAA};

        std::vector<std::size_t> offsets;
        auto it = data.begin();
        while (it != data.end())
        {
            it = std::search(it,
                             data.end(),
                             std::begin(kPrtHeadLE),
                             std::end(kPrtHeadLE));
            if (it == data.end())
            {
                break;
            }
            offsets.push_back(static_cast<std::size_t>(std::distance(data.begin(), it)));
            ++it;
        }

        return offsets;
    }

    std::size_t infer_prt_span(const std::vector<std::size_t> &offsets)
    {
        if (offsets.size() < 2)
        {
            throw std::runtime_error("not enough PRT heads found in prefix to infer PRT span");
        }

        std::unordered_map<std::size_t, std::size_t> hist;
        for (std::size_t idx = 1; idx < offsets.size(); ++idx)
        {
            const std::size_t diff = offsets[idx] - offsets[idx - 1];
            if (diff > radar::RadarConfig::kPrtHeaderBytes)
            {
                ++hist[diff];
            }
        }

        if (hist.empty())
        {
            throw std::runtime_error("failed to build a valid PRT span histogram");
        }

        std::size_t best_span = 0;
        std::size_t best_count = 0;
        for (const auto &entry : hist)
        {
            if (entry.second > best_count ||
                (entry.second == best_count && entry.first < best_span))
            {
                best_span = entry.first;
                best_count = entry.second;
            }
        }

        return best_span;
    }

    std::size_t round_up(std::size_t value, std::size_t align)
    {
        if (align == 0)
        {
            return value;
        }
        const std::size_t rem = value % align;
        return rem == 0 ? value : (value + align - rem);
    }

    PaddingCheckSummary inspect_padding_candidate(const std::vector<uint8_t> &prefix,
                                                  const std::vector<std::size_t> &offsets,
                                                  const radar::RadarConfig &cfg,
                                                  std::size_t payload_bytes,
                                                  std::size_t inferred_prt_span)
    {
        PaddingCheckSummary summary;
        summary.expected_signal_bytes = cfg.prt_data_bytes();
        summary.expected_total_bytes = radar::RadarConfig::kPrtHeaderBytes + summary.expected_signal_bytes;
        summary.expected_padded_span = round_up(summary.expected_total_bytes, payload_bytes);
        summary.expected_padding_bytes = summary.expected_padded_span - summary.expected_total_bytes;
        summary.span_matches = (summary.expected_padded_span == inferred_prt_span);

        if (summary.expected_padding_bytes == 0 || !summary.span_matches)
        {
            return summary;
        }

        const std::size_t inspect_blocks = std::min<std::size_t>(offsets.size(), 8);
        for (std::size_t idx = 0; idx < inspect_blocks; ++idx)
        {
            const std::size_t start = offsets[idx] + summary.expected_total_bytes;
            const std::size_t end = offsets[idx] + summary.expected_padded_span;
            if (end > prefix.size() || start >= end)
            {
                break;
            }

            const uint8_t *pad = prefix.data() + start;
            const std::size_t pad_len = end - start;

            if (summary.first_block_prefix.empty())
            {
                const std::size_t grab = std::min<std::size_t>(pad_len, 32);
                summary.first_block_prefix.assign(pad, pad + static_cast<std::ptrdiff_t>(grab));
            }

            ++summary.blocks_checked;
            summary.bytes_checked += pad_len;
            for (std::size_t i = 0; i < pad_len; ++i)
            {
                if (pad[i] == 0xFF)
                {
                    ++summary.ff_bytes;
                }
                else if (pad[i] == 0x00)
                {
                    ++summary.zero_bytes;
                }
                else
                {
                    ++summary.other_bytes;
                }
            }
        }

        return summary;
    }

    std::size_t overlap_len(std::size_t a_begin,
                            std::size_t a_end,
                            std::size_t b_begin,
                            std::size_t b_end)
    {
        const std::size_t begin = std::max(a_begin, b_begin);
        const std::size_t end = std::min(a_end, b_end);
        return end > begin ? (end - begin) : 0;
    }

    std::string hex_preview(const uint8_t *data, std::size_t len)
    {
        std::ostringstream oss;
        for (std::size_t i = 0; i < len; ++i)
        {
            if (i != 0)
            {
                oss << ' ';
            }
            oss << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned>(data[i]);
        }
        return oss.str();
    }

    void print_prt_packet_breakdown(const std::vector<uint8_t> &prefix,
                                    const std::vector<std::size_t> &offsets,
                                    const radar::RadarConfig &cfg,
                                    std::size_t payload_bytes,
                                    std::size_t inspect_prt_index,
                                    std::size_t inspect_hex_bytes)
    {
        std::cout << "\n[PRT Packet Inspect]\n";

        if (inspect_prt_index >= offsets.size())
        {
            std::cout << "  requested prt index " << inspect_prt_index
                      << " is outside scanned prefix (found " << offsets.size() << " heads)\n";
            return;
        }

        const std::size_t prt_begin = offsets[inspect_prt_index];
        const std::size_t signal_begin = prt_begin + radar::RadarConfig::kPrtHeaderBytes;
        const std::size_t signal_end = signal_begin + cfg.prt_data_bytes();
        const std::size_t prt_end = prt_begin + round_up(signal_end - prt_begin, payload_bytes);
        const std::size_t pad_begin = signal_end;
        const std::size_t pad_end = prt_end;

        if (prt_end > prefix.size())
        {
            std::cout << "  PRT#" << inspect_prt_index
                      << " exceeds scanned prefix, increase --scan-bytes if needed\n";
            return;
        }

        const radar::PrtHeaderInfo hdr =
            radar::decode_prt_header(prefix.data() + static_cast<std::ptrdiff_t>(prt_begin));
        const std::size_t packets = (prt_end - prt_begin) / payload_bytes;

        std::cout << "  inspect prt index   = " << inspect_prt_index << "\n";
        std::cout << "  file offset         = " << prt_begin << "\n";
        std::cout << "  global_prt_cnt      = " << hdr.global_prt_count() << "\n";
        std::cout << "  prt_num             = " << hdr.prt_num() << "\n";
        std::cout << "  packet count        = " << packets << "\n";
        std::cout << "  header bytes        = " << radar::RadarConfig::kPrtHeaderBytes << "\n";
        std::cout << "  signal bytes        = " << cfg.prt_data_bytes() << "\n";
        std::cout << "  candidate pad bytes = " << (pad_end - pad_begin) << "\n";

        for (std::size_t pkt = 0; pkt < packets; ++pkt)
        {
            const std::size_t pkt_begin = prt_begin + pkt * payload_bytes;
            const std::size_t pkt_end = pkt_begin + payload_bytes;
            const std::size_t hdr_bytes = overlap_len(pkt_begin, pkt_end, prt_begin, signal_begin);
            const std::size_t sig_bytes = overlap_len(pkt_begin, pkt_end, signal_begin, signal_end);
            const std::size_t pad_bytes = overlap_len(pkt_begin, pkt_end, pad_begin, pad_end);

            const uint8_t *pkt_ptr = prefix.data() + static_cast<std::ptrdiff_t>(pkt_begin);
            const std::size_t preview = std::min<std::size_t>(inspect_hex_bytes, payload_bytes);

            std::cout << "  packet[" << pkt << "]"
                      << " offset=" << pkt_begin
                      << " hdr=" << hdr_bytes
                      << " signal=" << sig_bytes
                      << " pad=" << pad_bytes << "\n";
            std::cout << "    first bytes: " << hex_preview(pkt_ptr, preview) << "\n";
            std::cout << "    last bytes : "
                      << hex_preview(pkt_ptr + static_cast<std::ptrdiff_t>(payload_bytes - preview), preview)
                      << "\n";
        }

        if (pad_end > pad_begin)
        {
            const uint8_t *pad_ptr = prefix.data() + static_cast<std::ptrdiff_t>(pad_begin);
            const std::size_t preview = std::min<std::size_t>(inspect_hex_bytes, pad_end - pad_begin);
            std::cout << "  candidate pad first bytes = " << hex_preview(pad_ptr, preview) << "\n";
            std::cout << "  candidate pad last bytes  = "
                      << hex_preview(pad_ptr + static_cast<std::ptrdiff_t>((pad_end - pad_begin) - preview), preview)
                      << "\n";
        }
    }

    void print_header_preview(const radar::PrtHeaderInfo &hdr)
    {
        std::cout << "    fixed_head      = 0x" << std::hex << std::setw(8) << std::setfill('0')
                  << hdr.fixed_head() << std::dec << std::setfill(' ') << "\n";
        std::cout << "    global_prt_cnt  = " << hdr.global_prt_count() << "\n";
        std::cout << "    prt_num         = " << hdr.prt_num() << "\n";
        std::cout << "    prt_samples     = " << hdr.prt_samples() << "\n";
        std::cout << "    frame_length    = " << hdr.frame_length_bytes() << "\n";
        std::cout << "    radar_mode      = " << hdr.radar_mode() << "\n";
        std::cout << "    data_type       = " << hdr.data_type() << "\n";
        std::cout << "    data_format     = " << hdr.data_format_bits() << "\n";
        std::cout << "    cfar_threshold  = " << hdr.cfar_threshold() << "\n";
        std::cout << "    radar_angle_a/e = " << hdr.radar_angle_a_raw() << " / "
                  << hdr.radar_angle_e_raw()
                  << " -> " << hdr.radar_angle_a_deg()
                  << " deg / " << hdr.radar_angle_e_deg() << " deg\n";
        std::cout << "    raw_word[19]    = 0x" << std::hex << std::setw(8) << std::setfill('0')
                  << hdr.words[18] << "\n";
        std::cout << "    raw_word[20]    = 0x" << std::setw(8)
                  << hdr.words[19] << std::dec << std::setfill(' ') << "\n";
    }

    void finalize_cpi(std::optional<CpiSummary> &active,
                      std::vector<CpiSummary> &preview,
                      std::vector<std::size_t> &cpi_lengths,
                      BeamStats &beam_stats,
                      VerifyStats &stats,
                      const VerifyOptions &opt)
    {
        if (!active.has_value())
        {
            return;
        }

        active->cpi_index = stats.cpi_count;
        ++stats.cpi_count;
        cpi_lengths.push_back(active->prt_count);

        const uint32_t beam_key =
            (static_cast<uint32_t>(active->first_header.radar_angle_a_raw()) << 16) |
            static_cast<uint32_t>(active->first_header.radar_angle_e_raw());
        ++beam_stats.beam_hist[beam_key];

        if (active->first_header.radar_angle_a_raw() == 1000 &&
            active->first_header.radar_angle_e_raw() == 1000)
        {
            ++beam_stats.zero_zero_count;
            if (beam_stats.zero_zero_cpi_indices.size() < 16)
            {
                beam_stats.zero_zero_cpi_indices.push_back(active->cpi_index);
                beam_stats.zero_zero_global_starts.push_back(active->global_start);
            }
        }

        if (preview.size() < opt.preview_cpi)
        {
            preview.push_back(*active);
        }

        active.reset();
    }

    VerifyStats verify_stream(const VerifyOptions &opt,
                              std::size_t inferred_prt_span,
                              std::vector<CpiSummary> &preview,
                              std::vector<std::size_t> &cpi_lengths,
                              BeamStats &beam_stats)
    {
        std::ifstream in(opt.input_path, std::ios::binary);
        if (!in)
        {
            throw std::runtime_error("failed to open input file: " + opt.input_path);
        }

        VerifyStats stats;
        std::vector<uint8_t> packet(opt.payload_bytes);
        std::vector<uint8_t> stream_buf;
        stream_buf.reserve(inferred_prt_span + opt.payload_bytes);

        std::optional<CpiSummary> active;
        uint32_t expected_next_global = 0;
        bool has_expected_global = false;

        while (in)
        {
            in.read(reinterpret_cast<char *>(packet.data()),
                    static_cast<std::streamsize>(packet.size()));
            const std::size_t got = static_cast<std::size_t>(in.gcount());
            if (got == 0)
            {
                break;
            }

            ++stats.udp_packets;
            stream_buf.insert(stream_buf.end(), packet.begin(), packet.begin() + static_cast<std::ptrdiff_t>(got));

            while (true)
            {
                const std::size_t before_resync = stream_buf.size();
                radar::resync_to_prt_head(stream_buf);
                stats.resync_drop_bytes += (before_resync - stream_buf.size());

                if (stream_buf.size() < inferred_prt_span)
                {
                    break;
                }

                if (!radar::starts_with_prt_head(stream_buf.data()))
                {
                    ++stats.invalid_head_count;
                    stream_buf.erase(stream_buf.begin());
                    continue;
                }

                const radar::PrtHeaderInfo hdr = radar::decode_prt_header(stream_buf.data());
                ++stats.prt_count;

                if (has_expected_global && hdr.global_prt_count() != expected_next_global)
                {
                    ++stats.global_gap_count;
                }
                expected_next_global = hdr.global_prt_count() + 1;
                has_expected_global = true;

                if (!active.has_value())
                {
                    active = CpiSummary{};
                    active->prt_count = 1;
                    active->first_prt_num = hdr.prt_num();
                    active->last_prt_num = hdr.prt_num();
                    active->global_start = hdr.global_prt_count();
                    active->global_end = hdr.global_prt_count();
                    active->first_header = hdr;
                }
                else
                {
                    if (hdr.prt_num() <= active->last_prt_num)
                    {
                        finalize_cpi(active, preview, cpi_lengths, beam_stats, stats, opt);

                        if (opt.max_cpi != 0 && stats.cpi_count >= opt.max_cpi)
                        {
                            return stats;
                        }

                        active = CpiSummary{};
                        active->prt_count = 1;
                        active->first_prt_num = hdr.prt_num();
                        active->last_prt_num = hdr.prt_num();
                        active->global_start = hdr.global_prt_count();
                        active->global_end = hdr.global_prt_count();
                        active->first_header = hdr;
                    }
                    else
                    {
                        if (hdr.prt_num() != static_cast<uint16_t>(active->last_prt_num + 1))
                        {
                            ++stats.prt_gap_count;
                        }
                        ++active->prt_count;
                        active->last_prt_num = hdr.prt_num();
                        active->global_end = hdr.global_prt_count();
                    }
                }

                stream_buf.erase(stream_buf.begin(),
                                 stream_buf.begin() + static_cast<std::ptrdiff_t>(inferred_prt_span));
            }
        }

        finalize_cpi(active, preview, cpi_lengths, beam_stats, stats, opt);
        stats.trailing_bytes = stream_buf.size();
        return stats;
    }

    std::size_t dominant_cpi_prt_count(const std::vector<std::size_t> &cpi_lengths)
    {
        std::unordered_map<std::size_t, std::size_t> hist;
        for (const std::size_t len : cpi_lengths)
        {
            ++hist[len];
        }

        std::size_t best_len = 0;
        std::size_t best_count = 0;
        for (const auto &entry : hist)
        {
            if (entry.second > best_count ||
                (entry.second == best_count && entry.first > best_len))
            {
                best_len = entry.first;
                best_count = entry.second;
            }
        }
        return best_len;
    }

    void print_padding_check(const PaddingCheckSummary &padding)
    {
        std::cout << "\n[Padding Hypothesis]\n";
        std::cout << "  expected signal bytes  = " << padding.expected_signal_bytes << "\n";
        std::cout << "  expected total bytes   = " << padding.expected_total_bytes << " (header + signal)\n";
        std::cout << "  expected padded span   = " << padding.expected_padded_span << "\n";
        std::cout << "  expected padding bytes = " << padding.expected_padding_bytes << "\n";
        std::cout << "  span matches observed  = " << (padding.span_matches ? "yes" : "no") << "\n";

        if (padding.blocks_checked == 0)
        {
            return;
        }

        std::cout << "  inspected blocks       = " << padding.blocks_checked << "\n";
        std::cout << "  inspected pad bytes    = " << padding.bytes_checked << "\n";
        std::cout << "  ff bytes               = " << padding.ff_bytes << "\n";
        std::cout << "  zero bytes             = " << padding.zero_bytes << "\n";
        std::cout << "  other bytes            = " << padding.other_bytes << "\n";
        if (!padding.first_block_prefix.empty())
        {
            std::cout << "  first pad bytes        =";
            for (uint8_t b : padding.first_block_prefix)
            {
                std::cout << " " << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<unsigned>(b);
            }
            std::cout << std::dec << std::setfill(' ') << "\n";
        }
    }

    void print_config_compare(const radar::RadarConfig &cfg,
                              std::size_t inferred_prt_span,
                              const std::vector<CpiSummary> &preview,
                              const std::vector<std::size_t> &cpi_lengths,
                              const PaddingCheckSummary &padding)
    {
        const std::size_t observed_head_to_head_payload =
            inferred_prt_span > radar::RadarConfig::kPrtHeaderBytes
                ? (inferred_prt_span - radar::RadarConfig::kPrtHeaderBytes)
                : 0;
        const std::size_t dominant_cpi_len = dominant_cpi_prt_count(cpi_lengths);

        std::cout << "\n[Config Compare]\n";
        std::cout << "  configured num_chirps  = " << cfg.num_chirps << "\n";
        if (dominant_cpi_len != 0)
        {
            std::cout << "  dominant observed CPI prts = " << dominant_cpi_len << "\n";
        }
        if (!preview.empty() && preview.front().prt_count != dominant_cpi_len)
        {
            std::cout << "  first previewed CPI is partial = " << preview.front().prt_count << "\n";
        }
        std::cout << "  configured num_samples = " << cfg.num_samples << "\n";
        std::cout << "  observed PRT span      = " << inferred_prt_span << "\n";
        std::cout << "  observed head-to-head payload = " << observed_head_to_head_payload << "\n";
        std::cout << "  configured signal bytes      = " << cfg.prt_data_bytes() << "\n";
        if (padding.span_matches)
        {
            std::cout << "  configured padding bytes     = " << padding.expected_padding_bytes << "\n";
        }
    }

    void print_beam_stats(const BeamStats &beam_stats)
    {
        std::vector<std::pair<uint32_t, std::size_t>> hist(beam_stats.beam_hist.begin(),
                                                           beam_stats.beam_hist.end());
        std::sort(hist.begin(), hist.end(),
                  [](const auto &lhs, const auto &rhs)
                  {
                      if (lhs.second != rhs.second)
                      {
                          return lhs.second > rhs.second;
                      }
                      return lhs.first < rhs.first;
                  });

        std::cout << "\n[Beam Summary]\n";
        std::cout << "  unique beam pairs   = " << hist.size() << "\n";
        std::cout << "  zero/zero beam cpis = " << beam_stats.zero_zero_count << "\n";
        if (!beam_stats.zero_zero_cpi_indices.empty())
        {
            std::cout << "  first zero/zero cpis:\n";
            for (std::size_t i = 0; i < beam_stats.zero_zero_cpi_indices.size(); ++i)
            {
                std::cout << "    cpi=" << beam_stats.zero_zero_cpi_indices[i]
                          << " global_start=" << beam_stats.zero_zero_global_starts[i] << "\n";
            }
        }

        const std::size_t preview_count = std::min<std::size_t>(hist.size(), 12);
        if (preview_count == 0)
        {
            return;
        }

        std::cout << "  top beam pairs:\n";
        for (std::size_t i = 0; i < preview_count; ++i)
        {
            const uint16_t raw_a = static_cast<uint16_t>(hist[i].first >> 16);
            const uint16_t raw_e = static_cast<uint16_t>(hist[i].first & 0xFFFFu);
            std::cout << "    raw=(" << raw_a << "," << raw_e << ")"
                      << " deg=(" << radar::decode_radar_angle_deg(raw_a)
                      << "," << radar::decode_radar_angle_deg(raw_e) << ")"
                      << " count=" << hist[i].second << "\n";
        }
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        const VerifyOptions opt = parse_options(argc, argv);
        const std::vector<uint8_t> prefix = load_prefix(opt.input_path, opt.scan_bytes);
        const std::vector<std::size_t> offsets = find_prt_offsets(prefix);
        const std::size_t inferred_prt_span = infer_prt_span(offsets);
        const radar::RadarConfig cfg;
        const PaddingCheckSummary padding =
            inspect_padding_candidate(prefix, offsets, cfg, opt.payload_bytes, inferred_prt_span);

        std::vector<CpiSummary> preview;
        std::vector<std::size_t> cpi_lengths;
        BeamStats beam_stats;
        const VerifyStats stats = verify_stream(opt, inferred_prt_span, preview, cpi_lengths, beam_stats);

        std::cout << "[Offline UDP Verify]\n";
        std::cout << "  input file         = " << opt.input_path << "\n";
        std::cout << "  payload bytes      = " << opt.payload_bytes << "\n";
        std::cout << "  scanned prefix     = " << prefix.size() << " bytes\n";
        std::cout << "  head hits in scan  = " << offsets.size() << "\n";
        if (!offsets.empty())
        {
            std::cout << "  first head offset  = " << offsets.front() << "\n";
        }
        std::cout << "  inferred PRT span  = " << inferred_prt_span
                  << " bytes (" << (inferred_prt_span / opt.payload_bytes)
                  << " x UDP payload)\n";

        std::cout << "\n[Stream Stats]\n";
        std::cout << "  udp packets        = " << stats.udp_packets << "\n";
        std::cout << "  prt count          = " << stats.prt_count << "\n";
        std::cout << "  cpi count          = " << stats.cpi_count << "\n";
        std::cout << "  resync drop bytes  = " << stats.resync_drop_bytes << "\n";
        std::cout << "  invalid heads      = " << stats.invalid_head_count << "\n";
        std::cout << "  prt gaps           = " << stats.prt_gap_count << "\n";
        std::cout << "  global gaps        = " << stats.global_gap_count << "\n";
        std::cout << "  trailing bytes     = " << stats.trailing_bytes << "\n";

        std::cout << "\n[CPI Preview]\n";
        if (preview.empty())
        {
            std::cout << "  no complete CPI detected\n";
        }
        else
        {
            for (const CpiSummary &cpi : preview)
            {
                std::cout << "  CPI#" << cpi.cpi_index
                          << " prt_count=" << cpi.prt_count
                          << " prt_range=[" << cpi.first_prt_num << ", " << cpi.last_prt_num << "]"
                          << " global=[" << cpi.global_start << ", " << cpi.global_end << "]\n";
                std::cout << "    first PRT header snapshot:\n";
                print_header_preview(cpi.first_header);
            }
        }

        print_padding_check(padding);
        print_config_compare(cfg, inferred_prt_span, preview, cpi_lengths, padding);
        print_beam_stats(beam_stats);
        if (opt.inspect_prt_index != std::numeric_limits<std::size_t>::max())
        {
            print_prt_packet_breakdown(prefix,
                                       offsets,
                                       cfg,
                                       opt.payload_bytes,
                                       opt.inspect_prt_index,
                                       opt.inspect_hex_bytes);
        }
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return 1;
    }
}
