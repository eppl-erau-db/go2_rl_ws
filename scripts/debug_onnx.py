import onnxruntime as ort

# Check available providers
providers = ort.get_available_providers()
print("Available providers:", providers)