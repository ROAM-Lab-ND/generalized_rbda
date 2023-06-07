function gen_code(f, name, opts)
    name = [name, '.cpp'];
    f.generate(name, opts);
    movefile(name, ['../source/' name]);
    movefile([name(1:end - 3) 'h'], ['../header/' name(1:end - 3) 'h']);
end
