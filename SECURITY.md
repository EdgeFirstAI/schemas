# Security Policy

## Supported Versions

EdgeFirst Perception Schemas follows semantic versioning. Security updates are provided for the following versions:

| Version | Supported          |
| ------- | ------------------ |
| 0.0.x   | :white_check_mark: |

**Note:** As this is a pre-1.0 release, the API may change. Once 1.0 is released, we will maintain backwards compatibility and provide security updates for the current major version.

## Reporting a Vulnerability

We take the security of EdgeFirst Perception Schemas seriously. If you discover a security vulnerability, please report it responsibly.

### How to Report

**DO NOT** report security vulnerabilities through public GitHub issues.

Instead, please use one of the following methods:

1. **Email**: Send details to support@au-zone.com with subject line: "Security Vulnerability - EdgeFirst Perception Schemas"
2. **GitHub Security Advisory**: Use the [private reporting feature](https://github.com/EdgeFirstAI/schemas/security/advisories/new)

### Information to Include

Help us understand and resolve the issue quickly by providing:

- **Type of vulnerability** (e.g., buffer overflow, injection, authentication bypass)
- **Full paths** of affected source files
- **Location of the affected code** (tag/branch/commit or direct URL)
- **Step-by-step instructions** to reproduce the issue
- **Proof-of-concept or exploit code** (if possible)
- **Impact assessment** (what an attacker could achieve)
- **Suggested fix** (if you have one)

### What to Expect

After you submit a vulnerability report:

1. **Acknowledgment**: We will acknowledge receipt within **48 hours**
2. **Initial Assessment**: We will provide an initial assessment within **7 days**, including:
   - Confirmation of the vulnerability
   - Severity classification (Critical, High, Medium, Low)
   - Estimated timeline for a fix
3. **Resolution Timeline**:
   - **Critical**: Fix within 7 days
   - **High**: Fix within 30 days
   - **Medium**: Fix within 90 days
   - **Low**: Fix in next scheduled release
4. **Disclosure**: We will work with you on responsible disclosure timing

### Vulnerability Severity

We use the [CVSS v3.1](https://www.first.org/cvss/v3.1/specification-document) scoring system:

- **Critical (9.0-10.0)**: Remote code execution, authentication bypass
- **High (7.0-8.9)**: Significant data disclosure, privilege escalation
- **Medium (4.0-6.9)**: Denial of service, limited information disclosure
- **Low (0.1-3.9)**: Minor information leak, low-impact issues

## Responsible Disclosure

We kindly ask security researchers to:

- Give us reasonable time to address the issue before public disclosure
- Make a good faith effort to avoid privacy violations, data destruction, and service disruption
- Not access, modify, or delete data without explicit permission
- Not exploit the vulnerability beyond what is necessary to demonstrate it

### Recognition

We appreciate responsible disclosure and will:

- Credit you in the security advisory (if desired)
- Include your name in our acknowledgments
- Provide Au-Zone branded swag for significant findings (subject to approval)

**Hall of Fame**: Contributors who responsibly disclose vulnerabilities will be recognized in this document.

## Security Update Process

When a security issue is confirmed:

1. **Patch Development**: Fix is developed in a private repository
2. **Testing**: Comprehensive testing including security validation
3. **Security Advisory**: GitHub Security Advisory created with CVE if applicable
4. **Release**: Patched version released with security notes
5. **Notification**:
   - GitHub Security Advisory notification
   - Email to known users (if applicable)
   - Release notes highlighting security fixes

## Security Best Practices for Users

To use EdgeFirst Perception Schemas securely:

### Message Validation

- Always validate message sizes before deserializing
- Check field counts and array lengths
- Implement timeout limits for message processing
- Validate data ranges for numeric fields

**Example (Rust):**

```rust
use edgefirst_schemas::sensor_msgs::PointCloud2;

fn safe_decode(pcd: &PointCloud2) -> Result<(), Error> {
    // Validate dimensions
    if pcd.height > 10_000 || pcd.width > 10_000 {
        return Err(Error::ExcessiveDimensions);
    }

    // Validate data size
    let expected_size = (pcd.height * pcd.width * pcd.point_step) as usize;
    if pcd.data.len() != expected_size {
        return Err(Error::DataSizeMismatch);
    }

    // Safe to decode
    Ok(())
}
```

### Network Security

- Use TLS/SSL for Zenoh connections in production
- Implement authentication for Zenoh routers
- Apply firewall rules to limit Zenoh access
- Monitor for unusual message patterns

### Dependency Management

- Regularly update EdgeFirst Perception Schemas to latest version
- Monitor security advisories for dependencies
- Use `cargo audit` (Rust) or `safety` (Python) to check for known vulnerabilities
- Review SBOM (`sbom.json`) for dependency licenses and versions

**Automated Check:**

```bash
# Rust
cargo install cargo-audit
cargo audit

# Python
pip install safety
safety check
```

## Known Security Considerations

### Deserialization

Message deserialization involves parsing untrusted data. While we implement bounds checking, users should:

- Set maximum message sizes
- Implement rate limiting
- Validate data before use in critical operations
- Use sandboxing for untrusted sources

### DMA Buffer Sharing

The `DmaBuffer` message type enables zero-copy buffer sharing but requires careful handling:

- Validate buffer file descriptors
- Implement access controls
- Clear sensitive data after use
- Don't expose DMA buffers to untrusted processes

### ROS2 Bridge

When using the Zenoh ROS2 DDS Bridge:

- ROS2 DDS has known security limitations
- Use ROS2 SROS2 (Secure ROS2) in production
- Implement network segmentation
- Monitor bridge traffic for anomalies

## Compliance

EdgeFirst Perception Schemas complies with:

- **SBOM Generation**: Complete Software Bill of Materials available
- **License Compliance**: Apache-2.0 licensed with compatible dependencies
- **Code Scanning**: Automated security scanning in CI/CD
- **Dependency Auditing**: Regular dependency vulnerability checks

## Enterprise Security Services

Au-Zone Technologies offers additional security services for production deployments:

- **Security Audits**: Professional code review and penetration testing
- **Compliance Certification**: Help achieving industry certifications (e.g., IEC 62443)
- **Custom Security Features**: Tailored security implementations
- **Incident Response**: 24/7 support for security incidents
- **Security Training**: Team training on secure coding practices

**Contact:** support@au-zone.com | **Enterprise:** https://au-zone.com/contact

## Security Resources

- **OWASP Top 10**: https://owasp.org/www-project-top-ten/
- **CWE List**: https://cwe.mitre.org/
- **NIST NVD**: https://nvd.nist.gov/
- **Rust Security Database**: https://rustsec.org/
- **GitHub Security Lab**: https://securitylab.github.com/

---

**Last Updated:** November 17, 2025

For general questions, see our [Contributing Guide](CONTRIBUTING.md) or visit [EdgeFirst Documentation](https://doc.edgefirst.ai/latest/perception/).
