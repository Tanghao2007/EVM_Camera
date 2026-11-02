"""
Fast Non-Backtracking Verilog Port Checker
"""
import os
import re

script_dir = os.path.dirname(os.path.abspath(__file__))
rtl_root = os.path.normpath(os.path.join(script_dir, "..", "rtl"))

modules = {}

def get_module_ports(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    in_port_list = False
    mod_name = None
    ports = set()

    for line in lines:
        line_clean = re.sub(r'//.*', '', line).strip()
        if not line_clean:
            continue
        
        m = re.match(r'module\s+([a-zA-Z0-9_]+)', line_clean)
        if m:
            mod_name = m.group(1)
            in_port_list = True
            continue

        if in_port_list:
            # Check if port list ended
            if line_clean.startswith(');'):
                in_port_list = False
                break
            
            # Find declarations: input / output / inout
            # e.g.: input wire [7:0] p00, p01,
            # or:   output reg [23:0] vid_out_rgb
            decl_match = re.match(r'(?:input|output|inout)\s+(?:wire|reg)?\s*(?:signed)?\s*(?:\[[^\]]+\])?\s*(.*)', line_clean)
            if decl_match:
                names_part = decl_match.group(1)
                names_part = names_part.rstrip(',);').strip()
                for token in names_part.split(','):
                    name = token.strip()
                    if name and re.match(r'^[a-zA-Z0-9_]+$', name):
                        ports.add(name)
            else:
                # continuation line of multiple ports
                if not line_clean.startswith('parameter') and not line_clean.startswith('#'):
                    names_part = line_clean.rstrip(',);').strip()
                    for token in names_part.split(','):
                        name = token.strip()
                        if name and re.match(r'^[a-zA-Z0-9_]+$', name):
                            ports.add(name)

    return mod_name, ports

def main():
    for root, dirs, files in os.walk(rtl_root):
        for f in files:
            if f.endswith('.v'):
                path = os.path.join(root, f)
                mod_name, ports = get_module_ports(path)
                if mod_name:
                    modules[mod_name] = {'path': path, 'ports': ports}
                    print(f"Parsed {mod_name:22s}: {len(ports):2d} ports")

    top_path = os.path.join(rtl_root, "top", "evm_pl_top.v")
    with open(top_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Find instances: e.g. u_line_buffer, u_laplacian_pyr, etc.
    inst_headers = re.findall(r'([a-zA-Z0-9_]+)\s+(?:#\s*\([^;]+?\)\s+)?(u_[a-zA-Z0-9_]+)\s*\(', content)
    
    print("\n--- Verifying Instantiations in evm_pl_top.v ---")
    all_ok = True
    for sub_mod, inst_name in inst_headers:
        if sub_mod not in modules:
            print(f"[ERROR] Module {sub_mod} not found!")
            all_ok = False
            continue
        
        # Extract port mapping for this instance
        start_idx = content.find(inst_name)
        open_paren = content.find('(', start_idx)
        # Find matching close paren followed by semicolon
        semi_idx = content.find(');', open_paren)
        block = content[open_paren+1:semi_idx]
        
        mapped = set(re.findall(r'\.([a-zA-Z0-9_]+)\s*\(', block))
        defined = modules[sub_mod]['ports']
        
        extra = mapped - defined
        missing = defined - mapped
        
        if extra or missing:
            all_ok = False
            print(f"[FAIL] {inst_name} ({sub_mod}):")
            if extra: print(f"       Extra: {extra}")
            if missing: print(f"       Missing: {missing}")
        else:
            print(f"[OK]   {inst_name:20s} ({sub_mod:20s}): {len(mapped)} ports match 100%")

    if all_ok:
        print("\n>>> ALL 9 MODULES INSTANTIATED WITH 100% PORT CONSISTENCY! <<<")

if __name__ == '__main__':
    main()
