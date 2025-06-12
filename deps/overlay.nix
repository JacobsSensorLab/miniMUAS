final: prev: {
  openabe = final.callPackage ./openabe {};
  
  ndn-cxx = final.callPackage ./ndn-cxx {};

  ndn-svs = final.callPackage ./ndn-svs {};

  ndnsd = final.callPackage ./ndnsd {};

  nac-abe = final.callPackage ./nac-abe {};

  ndnsf = final.callPackage ./ndnsf {};

  mavsdk = final.callPackage ./mavsdk {};

  mavlink = final.callPackage ./mavlink {};

  tinyxml2 = final.callPackage ./tinyxml2 {};
}
