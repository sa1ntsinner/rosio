;;; rosio.el --- rosio-lsp for Emacs  -*- lexical-binding: t; -*-

;; Load this file from your init file. Each block takes effect only once
;; its package loads, so it works with lsp-mode, Eglot or both.
;;
;; lsp-mode runs rosio as an add-on, beside clangd, pylsp and the rest.
;; Eglot runs one server per buffer, so the Eglot block covers only yaml
;; and xml buffers, where rosio takes the place of yaml-language-server
;; or lemminx. Findings in C++ and Python files need lsp-mode.

(with-eval-after-load 'lsp-mode
  (lsp-register-client
   (make-lsp-client
    :new-connection (lsp-stdio-connection '("rosio-lsp"))
    :activation-fn (lsp-activate-on "c" "cpp" "python" "yaml" "xml" "cmake")
    :add-on? t
    :server-id 'rosio
    :initialization-options '(:atlasPath "rosio-out/atlas.json"
                              :extractCommand ["rosio" "extract" "."]
                              :extractOnSave t))))

(with-eval-after-load 'eglot
  (add-to-list 'eglot-server-programs
               '((yaml-mode yaml-ts-mode nxml-mode)
                 . ("rosio-lsp" :initializationOptions
                    (:atlasPath "rosio-out/atlas.json"
                     :extractCommand ["rosio" "extract" "."]
                     :extractOnSave t)))))

;;; rosio.el ends here
