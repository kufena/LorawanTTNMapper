//
// This is the decoder used by The Things Network application to decode
// the bytes from received messages.
//
// The decoded JSON can then be passed to the TTN Mapper integration,
// although I don't think "sats" is what it expects - it's the level of
// fix the GPS transceiver has, not the number of satellites.
//
function Decoder(bytes, port) {
  
  fix = bytes[0];
  lon = ((bytes[1] & 0xFF) | ((bytes[2] & 0xFF) << 8) | ((bytes[3] & 0xFF) << 16) | ((bytes[4] & 0xFF) << 24));
  lat = ((bytes[5] & 0xFF) | ((bytes[6] & 0xFF) << 8) | ((bytes[7] & 0xFF) << 16) | ((bytes[8] & 0xFF) << 24));
  alt = ((bytes[9] & 0xFF) | ((bytes[10] & 0xFF) << 8) | ((bytes[11] & 0xFF) << 16) | ((bytes[12] & 0xFF) << 24));
  vacc = ((bytes[13] & 0xFF) | ((bytes[14] & 0xFF) << 8) | ((bytes[15] & 0xFF) << 16) | ((bytes[16] & 0xFF) << 24));
  hacc = ((bytes[17] & 0xFF) | ((bytes[18] & 0xFF) << 8) | ((bytes[19] & 0xFF) << 16) | ((bytes[20] & 0xFF) << 24));
  
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {
    "longitude": lon/10000000,
    "latitude": lat/10000000,
    "altitude": alt / 1000,
    "accuracy": hacc,
    "vaccuracy": vacc,
    "sats": fix
  };

  // if (port === 1) decoded.led = bytes[0];

  return decoded;
}
