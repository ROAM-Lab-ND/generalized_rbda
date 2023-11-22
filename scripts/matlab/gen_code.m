function gen_code(f, name, opts)
    name = [name, '.cpp'];
    f.generate(name, opts);
    movefile(name, ['../../src/Codegen/' name]);
    movefile([name(1:end - 3) 'h'], ['../../include/grbda/Codegen/' name(1:end - 3) 'h']);
end
