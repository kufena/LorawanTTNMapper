function Decoder(bytes, port) {
  
  fix = bytes[0];
  fixok = bytes[29];
  lon = ((bytes[1] & 0xFF) | ((bytes[2] & 0xFF) << 8) | ((bytes[3] & 0xFF) << 16) | ((bytes[4] & 0xFF) << 24));
  lat = ((bytes[5] & 0xFF) | ((bytes[6] & 0xFF) << 8) | ((bytes[7] & 0xFF) << 16) | ((bytes[8] & 0xFF) << 24));
  alt = ((bytes[9] & 0xFF) | ((bytes[10] & 0xFF) << 8) | ((bytes[11] & 0xFF) << 16) | ((bytes[12] & 0xFF) << 24));
  vacc = ((bytes[13] & 0xFF) | ((bytes[14] & 0xFF) << 8) | ((bytes[15] & 0xFF) << 16) | ((bytes[16] & 0xFF) << 24));
  hacc = ((bytes[17] & 0xFF) | ((bytes[18] & 0xFF) << 8) | ((bytes[19] & 0xFF) << 16) | ((bytes[20] & 0xFF) << 24));
  hdop = ((bytes[23] & 0xFF) | ((bytes[24] & 0xFF) << 8));
  msl = ((bytes[25] & 0xFF) | ((bytes[26] & 0xFF) << 8) | ((bytes[27] & 0xFF) << 16) | ((bytes[28] & 0xFF) << 24));
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {
    "longitude": lon/10000000,
    "latitude": lat/10000000,
    "altitude": alt / 1000,
    "altitudeMSL": msl / 1000,
    "accuracy": hacc / 1000, // TTN mapper wants meters, not mm.
    "vaccuracy": vacc / 1000, // TTN mapper wants meters, not mm.
    "fix": fix,
    "fixok": fixok,
    "hdop": hdop / 100
  };

  // if (port === 1) decoded.led = bytes[0];

  return decoded;
}