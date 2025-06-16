#include <ndn-cxx/face.hpp>
#include <ndn-cxx/security/key-chain.hpp>


//#include <attribute-authority.hpp> // or <nac-abe/attribute-authority.hpp>
#include <nac-abe/attribute-authority.hpp>
#include <nac-abe/cache-producer.hpp>
#include <iostream>

#include <ndn-service-framework/ServiceController.hpp>

int
main(int argc, char** argv)
{
  if (argc != 2)
  {
      std::cerr << "Usage : service-controller <conf_dir>" << std::endl;
      exit(1);
  }

  std::string config_dir = argv[1];

  try {
    ndn::Face m_face;
    ndn::KeyChain m_keyChain;
    ndn::ValidatorConfig m_validator(m_face);
    ndn::security::Certificate m_aaCert(m_keyChain.getPib().getIdentity("/muas/aa").getDefaultKey().getDefaultCertificate());
    m_validator.load(config_dir + "/trust-any.conf");
    ndn_service_framework::ServiceController controller(m_face, m_aaCert, m_validator, config_dir + "/minimuas.policies");
    m_face.processEvents();
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "ERRAND: " << e.what() << std::endl;
    return 1;
  }
}