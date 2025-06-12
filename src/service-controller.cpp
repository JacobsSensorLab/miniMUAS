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
  try {
    ndn::Face m_face;
    ndn::KeyChain m_keyChain;
    ndn::ValidatorConfig m_validator(m_face);
    ndn::security::Certificate m_aaCert(m_keyChain.getPib().getIdentity("/muas/aa").getDefaultKey().getDefaultCertificate());
    m_validator.load("/usr/local/bin/trust-any.conf");
    ndn_service_framework::ServiceController controller(m_face, m_aaCert, m_validator, "/usr/local/bin/minimuas.policies");
    m_face.processEvents();
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "ERRAND: " << e.what() << std::endl;
    return 1;
  }
}