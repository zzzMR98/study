
#include "io/proto_stream.h"

namespace ivcommon {
namespace io {

namespace {

// First eight bytes to identify our proto stream format.
const uint64 kMagic = 0x7b1d1f7b5bf501db;
///
///以小端字节序写数据大小
///
void WriteSizeAsLittleEndian(uint64 size, std::ostream* out) {
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}
///
///以小端字节序读数据大小
///
bool ReadSizeAsLittleEndian(std::istream* in, uint64* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<uint64>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace

ProtoStreamWriter::ProtoStreamWriter(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {
  WriteSizeAsLittleEndian(kMagic, &out_);
}

ProtoStreamWriter::~ProtoStreamWriter() {}
///
///写数据
///
void ProtoStreamWriter::Write(const string& uncompressed_data) {
  string compressed_data;
  ::ivcommon::FastGzipString(uncompressed_data, &compressed_data);
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  out_.write(compressed_data.data(), compressed_data.size());
}
///
///关闭写操作
///
bool ProtoStreamWriter::Close() {
  out_.close();
  return !out_.fail();
}

ProtoStreamReader::ProtoStreamReader(const string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  uint64 magic;
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
}

ProtoStreamReader::~ProtoStreamReader() {}
///
///读数据
///
bool ProtoStreamReader::Read(string* decompressed_data) {
  uint64 compressed_size;
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  string compressed_data(compressed_size, '\0');
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  ::ivcommon::FastGunzipString(compressed_data, decompressed_data);
  return true;
}

bool ProtoStreamReader::eof() const { return in_.eof(); }

}  // namespace io
}